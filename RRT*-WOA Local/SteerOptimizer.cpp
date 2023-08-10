#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include "SteerOptimizer.hpp"
#include "MapProcessor.hpp"
#include "Logger.hpp"
#include "Objects.hpp"
#include "Debug.hpp"

extern nlohmann::json mySettings;
extern MapProcessor myMap;
extern cv::VideoWriter* myVideo;

State SteerOptimizer::Apply()
{
    InitializePopulation();
    CalculateFitness();
    while (this->iterations--)
    {
        for (this->i = 0; this->i < this->population; ++(this->i))
        {
            CoefficientUpdate();
            if ((this->p) < 0.5l)
            {
                if (fabs(this->A) < 1l)
                {
                    CircleUpdate();
                }
                else
                {
                    RandomUpdate();
                }
            }
            else
            {
                SpiralUpdate();
            }
        }
        CheckBoundary();
        CalculateFitness();
        RenderParticles();
    }
    CleanUp();
    return this->best;
}

void SteerOptimizer::InitializePopulation()
{
    this->particles = new State[this->population];
    for (int i = 0; i < this->population; ++i)
    {
        long double angle = GenerateRandom(0.0l, 2.0l * M_PI);
        //this->particles[i].x = GenerateRandom(this->original.x - this->widthBound, this->original.x + this->widthBound);
        //this->particles[i].y = GenerateRandom(this->original.y - this->heightBound, this->original.y + this->heightBound);
        this->particles[i].x = this->original.x + this->widthBound * std::cos(angle);
        this->particles[i].y = this->original.y + this->heightBound * std::sin(angle);
    }
    this->greatestFitness = std::numeric_limits<long double>::max();
}

void SteerOptimizer::CalculateFitness()
{
    this->bestFitness = std::numeric_limits<long double>::max();
    long double fitness, distanceToGoal, distanceToRand, visibility;
    for (int i = 0; i < this->population; ++i)
    {
        distanceToGoal = CalculateEuclideanDistance(this->particles[i], goal);
        distanceToRand = CalculateEuclideanDistance(this->particles[i], original);
        visibility = 0.0l;
        visibility += IsObstacleFree(this->particles[i], this->near) ? 0.0 : 1000.0;
        visibility += IsObstacleFree(this->particles[i], this->goal) ? 0.0 : 500.0;
        fitness =   distanceToGoalWeight * distanceToGoal +
                    distanceToRandWeight * distanceToRand + 
                    visibility;
        if (fitness < this->bestFitness)
        {
            this->bestFitness = fitness;
            this->best = this->particles[i];
        }
    }
    if (this->bestFitness < this->greatestFitness)
    {
        SaveGreatest();
        this->greatestFitness = this->bestFitness;
    }
    else
    {
        LoadGreatest();
    }
}

void SteerOptimizer::CoefficientUpdate()
{
    this->a -= (this->_a / (long double)(this->_iterations * this->population));
    this->r = GenerateRandom(0.0l, 1.0l);
    this->A = (2.0l * this->a * this->r) - this->a;
    this->C = (2.0l * this->r);
    this->l = GenerateRandom(-1.0l, 1.0l);
    this->p = GenerateRandom(0.0l, 1.0l);
}

void SteerOptimizer::CircleUpdate()
{
    long double Dx = fabs((this->C * this->best.x) - this->particles[this->i].x);
    long double Dy = fabs((this->C * this->best.y) - this->particles[this->i].y);
    this->particles[this->i].x = this->best.x - (this->A * Dx);
    this->particles[this->i].y = this->best.y - (this->A * Dy);
}

void SteerOptimizer::RandomUpdate()
{
    int index = GenerateRandom(0, this->population);
    this->random = this->particles[index];
    long double Dx = fabs((this->C * this->random.x) - this->particles[this->i].x);
    long double Dy = fabs((this->C * this->random.y) - this->particles[this->i].y);
    this->particles[this->i].x = this->random.x - (this->A * Dx);
    this->particles[this->i].y = this->random.y - (this->A * Dy);
}

void SteerOptimizer::SpiralUpdate()
{
    long double Dx = fabs(this->best.x - this->particles[this->i].x);
    long double Dy = fabs(this->best.y - this->particles[this->i].y);
    this->particles[this->i].x = Dx * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].x;
    this->particles[this->i].y = Dy * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].y;
}

void SteerOptimizer::CheckBoundary()
{
    for (int i = 0; i < this->population; ++i)
    {
        double distance = sqrt(pow(this->particles[i].x - this->original.x, 2)+ pow(this->particles[i].y - this->original.y, 2));
        if (distance > this->widthBound)
        {
            long double dx = this->particles[i].x - this->original.x;
            long double dy = this->particles[i].y - this->original.y;
            long double scale = this->widthBound / distance;
            this->particles[i].x = this->original.x + dx * scale;
            this->particles[i].y = this->original.y + dy * scale;


        }
    //     // Correct them back into the circle
    //     if (this->particles[i].x > (this->original.x + this->widthBound))
    //     {
    //         this->particles[i].x = (this->original.x + this->widthBound);
    //     }
    //     else if (this->particles[i].x < (this->original.x - this->widthBound))
    //     {
    //         this->particles[i].x = (this->original.x - this->widthBound);
    //     }
    //     if (this->particles[i].y > (this->original.y + this->heightBound))
    //     {
    //         this->particles[i].y = (this->original.y + this->heightBound);
    //     }
    //     else if (this->particles[i].y < (this->original.y - this->heightBound))
    //     {
    //         this->particles[i].y = (this->original.y - this->heightBound);
    //     }
    //     // Correct them back into the map
    //     if (this->particles[i].x > -myMap.origin.x)
    //     {
    //         this->particles[i].x = -myMap.origin.x;
    //     }
    //     else if (this->particles[i].x < myMap.origin.x)
    //     {
    //         this->particles[i].x = myMap.origin.x;
    //     }
    //     if (this->particles[i].y > -myMap.origin.y)
    //     {
    //         this->particles[i].y = -myMap.origin.y;
    //     }
    //     else if (this->particles[i].y < myMap.origin.y)
    //     {
    //         this->particles[i].y = myMap.origin.y;
    //     }
    // }
    }
}

void SteerOptimizer::RenderParticles()
{
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    image = myMap.colored.clone();
    /// Draw all particles.
    int x, y;
    std::vector<State> CantSeeNone; // Red
    std::vector<State> CanSeeNear;  // Blue
    std::vector<State> CanSeeGoal;  // Orange
    std::vector<State> CanSeeBoth;  // Green
    bool SeesNear, SeesGoal;
    for (int i = 0; i < this->population; ++i)
    {
        SeesNear = SeesGoal = false;
        if (IsObstacleFree(this->particles[i], this->near))
        {
            SeesNear = true;
        }
        if (IsObstacleFree(this->particles[i], this->goal))
        {
            SeesGoal = true;
        }
        if (!SeesNear && !SeesGoal)
        {
            CantSeeNone.push_back(this->particles[i]);
        }
        else if (SeesNear && !SeesGoal)
        {
            CanSeeNear.push_back(this->particles[i]);
        }
        else if (!SeesNear && SeesGoal)
        {
            CanSeeGoal.push_back(this->particles[i]);
        } 
        else
        {
            CanSeeBoth.push_back(this->particles[i]);
        }
    }
    // Draw Can't See None
    for (int i = 0; i < CantSeeNone.size(); ++i)
    {
        x = XConvertToPixel(CantSeeNone[i].x);
        y = YConvertToPixel(CantSeeNone[i].y);
        cv::Point p(x, y);
        circle(image, p, 3, cv::Scalar(0x00, 0x00, 0xFF), -1, cv::LINE_AA);
    }
    // Can See Near
    for (int i = 0; i < CanSeeNear.size(); ++i)
    {
        x = XConvertToPixel(CanSeeNear[i].x);
        y = YConvertToPixel(CanSeeNear[i].y);
        cv::Point p(x, y);
        circle(image, p, 3, cv::Scalar(0xFF, 0x00, 0x00), -1, cv::LINE_AA);
    }

    // Can See Goal
    for (int i = 0; i < CanSeeGoal.size(); ++i)
    {
        x = XConvertToPixel(CanSeeGoal[i].x);
        y = YConvertToPixel(CanSeeGoal[i].y);
        cv::Point p(x, y);
        circle(image, p, 3, cv::Scalar(0x00, 0xA5, 0xFF), -1, cv::LINE_AA);
    }

    // Can See Goal
    for (int i = 0; i < CanSeeBoth.size(); ++i)
    {
        x = XConvertToPixel(CanSeeBoth[i].x);
        y = YConvertToPixel(CanSeeBoth[i].y);
        cv::Point p(x, y);
        circle(image, p, 3, cv::Scalar(0x00, 0xFF, 0x00), -1, cv::LINE_AA);

    }
    /// Draw currently best.
    x = XConvertToPixel(this->best.x);
    y = YConvertToPixel(this->best.y);
    cv::Point p(x, y);
    circle(image, p, 5, cv::Scalar(0xFF, 0x00, 0xFF), -1, cv::LINE_AA);

    bool renderGraphics = mySettings["RenderGraphics"];
    if (renderGraphics)
    {
        cv::imshow("WOA", image);
        cv::waitKey(0);
    }
    bool createVideo = mySettings["CreateVideo"];
    if (createVideo)
    {
        myVideo->write(image);
    }
}

void SteerOptimizer::CleanUp()
{
    delete this->particles;
    return;
}

/// ------------------------------------------------------------------------------------ 

long double SteerOptimizer::GenerateRandom(long double a, long double b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution(a, b);
    return distribution(engine);
}

int SteerOptimizer::GenerateRandom(int a, int b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution((long double)a, (long double)b);
    long double random = distribution(engine);
    return (int)(std::floor(random));  
}

long double SteerOptimizer::CalculateEuclideanDistance(State& a, State& b)
{
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2));
}

int SteerOptimizer::XConvertToPixel(long double x)
{
    return (int)((x + (-myMap.origin.x)) / myMap.resolution);
}

int SteerOptimizer::YConvertToPixel(long double y)
{
    return (int)((y + (-myMap.origin.x)) / myMap.resolution);
}

bool SteerOptimizer::DoIntersect(cv::Point& p1, cv::Point& q1, cv::Point& p2, cv::Point& q2)
{
    int o1 = Orientation(p1, q1, p2);
    int o2 = Orientation(p1, q1, q2);
    int o3 = Orientation(p2, q2, p1);
    int o4 = Orientation(p2, q2, q1);
    if (o1 != o2 && o3 != o4)
        return true;
    if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
    if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
    if (o3 == 0 && OnSegment(p2, p1, q2)) return true;  
    if (o4 == 0 && OnSegment(p2, q1, q2)) return true;
    return false;
}

int SteerOptimizer::Orientation(cv::Point& p, cv::Point& q, cv::Point& r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool SteerOptimizer::OnSegment(cv::Point& p, cv::Point& q, cv::Point& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;
    return false;
}

bool SteerOptimizer::IsObstacleFree(State& sNew, State& sNeighbor)
{
    #ifdef DEBUG
        static Logger log("IsObstacleFree [Segment] ");
    #endif
    int px = XConvertToPixel(sNew.x);
    int py = YConvertToPixel(sNew.y);
    cv::Point p(px, py);
    int qx = XConvertToPixel(sNeighbor.x);
    int qy = YConvertToPixel(sNeighbor.y);
    cv::Point q(qx, qy);
    for (auto& segment: myMap.segments)
    {
        if (DoIntersect(p, q, segment.p, segment.q))
        {
            #ifdef DEBUG
                log.Trace("Segment p(x:{:.2f}, y:{:.2f}) q(x:{:.2f}, y:{:.2f}) is not free", sNew.x, sNew.y, sNeighbor.x, sNeighbor.y);
            #endif
            return false;
        }
    }
    #ifdef DEBUG
        log.Trace("Segment p(x:{:.2f}, y:{:.2f}) q(x:{:.2f}, y:{:.2f}) is free", sNew.x, sNew.y, sNeighbor.x, sNeighbor.y);
    #endif
    return true;
}

void SteerOptimizer::SaveGreatest()
{
    this->greatest = this->best;
}

void SteerOptimizer::LoadGreatest()
{
    this->best = this->greatest;
}

State SteerOptimizer::ApplyV2()
{
    InitializePopulation();
    CalculateFitness();
    while (this->iterations--)
    {
        if (this->iterations > this->_iterations*0.5)
        {
            for (this->i = 0; this->i < this->population; ++(this->i))
            {
                CoefficientUpdate();
                if (fabs(this->A) < 0.5l)
                {
                    CircleUpdate();
                }
                else
                {
                    RandomUpdate();
                }
            }
            CheckBoundary();
            CalculateFitness();
            RenderParticles();
        }
        else
        {
            for (this->i = 0; this->i < this->population; ++(this->i))
            {
                CoefficientUpdate();
                SpiralUpdate();
            }
            CheckBoundary();
            CalculateFitness();
            RenderParticles();   
        }
    }
    return this->best;
}

State SteerOptimizer::ApplyV3()
{
    InitializePopulation();
    CalculateFitness();
    int search = 1 * this->_iterations;
    int attack = 0 * this->_iterations;
    this->iterations = search;
    while (this->iterations--)
    {
            for (this->i = 0; this->i < this->population; ++(this->i))
            {
                CoefficientUpdate();
                if (fabs(this->A) < 0.5l)
                {
                    CircleUpdate();
                }
                else
                {
                    RandomUpdate();
                }
            }
            CheckBoundary();
            CalculateFitness();
            RenderParticles();
    }
    this->a = this->_a;
    this->iterations = attack;
    while (this->iterations--)
    {

        for (this->i = 0; this->i < this->population; ++(this->i))
        {
            CoefficientUpdate();
            SpiralUpdate();
        }
        CheckBoundary();
        CalculateFitness();
        RenderParticles();
    }
    return this->best;
}