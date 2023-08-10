#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include "PathOptimizer.hpp"
#include "MapProcessor.hpp"
#include "Logger.hpp"
#include "Objects.hpp"

extern nlohmann::json mySettings;
extern MapProcessor myMap;
extern cv::VideoWriter* myVideo1;

Path PathOptimizer::Apply()
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
                if (fabs(this->A) < 0.5l)
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
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    image = myMap.colored.clone();
    int x,y;
    for (int s = 1; s < this->greatest.size; ++s)
    {
        State a = this->greatest.array[s];
        State b = this->greatest.array[s-1];
        x = XConvertToPixel(a.x);
        y = YConvertToPixel(a.y);
        cv::Point p(x, y); 
        x = XConvertToPixel(b.x);
        y = YConvertToPixel(b.y);
        cv::Point q(x, y); 
        line(image, p, q, cv::Scalar(0x00, 0xFF, 0x00), 3, cv::LINE_AA);
    }
    for (int c = 0; c < 30; ++c)
        myVideo1->write(image);
     /// Removing duplicate states.
    std::vector<State> path;
    for (int i = 0; i < this->greatest.size; ++i)
    { 
        if (i == 0)
        {
            path.push_back(this->greatest.array[i]);
            continue;
        }
        else
        {
            bool condition1 = fabs(path.back().x - this->greatest.array[i].x) <= std::numeric_limits<long double>::epsilon();
            bool condition2 = fabs(path.back().y - this->greatest.array[i].y) <= std::numeric_limits<long double>::epsilon();
            if (condition1 && condition2)
            {
                continue;
            }
            else
            {
                path.push_back(this->greatest.array[i]);
            }
        }
    }
    /// adding intermediary points
    std::vector<State> newPath;
    for (int i = 1; i < path.size(); ++i)
    {
        State p1 = path[i-1];
        State p2 = path[i];
        long double distance = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
        if (distance > 1.5l)
        {
            int numIntermediaryPoints = static_cast<int>(std::ceil(distance / 1.5l)) - 1;
            double xStep = (p2.x - p1.x) / (numIntermediaryPoints + 1);
            double yStep = (p2.y - p1.y) / (numIntermediaryPoints + 1);
            newPath.push_back(p1);
            for (int j = 0; j < numIntermediaryPoints-1; ++j)
            {
                double newX = p1.x + (xStep * (j + 1));
                double newY = p1.y + (yStep * (j + 1));
                newPath.push_back(State (newX, newY));
            }     
        }
        else
        {
            newPath.push_back(p1);
        }
    }
    newPath.push_back(path.back());
    Path _path;
    _path.size = newPath.size();
    _path.array = new State[_path.size];
    for (int i = 0; i < _path.size; ++i)
    {
        _path.array[i] = newPath[i];
    }
    return _path;
}

void PathOptimizer::InitializePopulation()
{
    /// Allocate the population.
    this->particles = new Path[this->population];
    particles[0].size = this->size;
    particles[0].array = new State[this->size];
    /// Copy the original path.
    for (int i = 0; i < this->size; ++i)
    {
        particles[0].array[i] = original.array[i];
    }
    /// Generate the rest of the particles.
    for (int i = 1; i < this->population; ++i)
    {
        particles[i].size = this->size;
        particles[i].array = new State[this->size];
        for (int j = 0; j < this->size; ++j)
        {
            /// Initialize the first element.
            if (j == 0)
            {
                this->particles[i].array[j].x = this->original.array[j].x;
                this->particles[i].array[j].y = this->original.array[j].y;
                continue;
            }
            /// Initialize the last element.
            else if (j == this->size-1)
            {
                this->particles[i].array[j].x = this->original.array[j].x;
                this->particles[i].array[j].y = this->original.array[j].y;
                continue;
            }
            /// Initialize the rest of the particles.
            this->particles[i].array[j].x = GenerateRandom(this->original.array[j].x - this->widthBound, this->original.array[j].x + this->widthBound);
            this->particles[i].array[j].y = GenerateRandom(this->original.array[j].y - this->heightBound, this->original.array[j].y + this->heightBound);
        }
    }
    /// The greatest acts as a memory path.
    this->greatest.size = this->size;
    this->greatest.array = new State[this->size];
    this->greatestFitness = std::numeric_limits<long double>::max();
}

void PathOptimizer::CalculateFitness()
{
    static Logger log(__FUNCTION__);
    this->bestFitness = std::numeric_limits<long double>::max();
    long double fitness, shortness, collision;
    for (int i = 0; i < this->population; ++i)
    {
        shortness = 0.0l;
        collision = 0.0l;
        for (int j = 1; j < this->size; ++j)
        {
            shortness += CalculateEuclideanDistance(this->particles[i].array[j-1], this->particles[i].array[j]);
            collision += IsObstacleFree(this->particles[i].array[j-1], this->particles[i].array[j]) ? 0.0l : 999.0l;
        }
        fitness = pathShortnessWeight * shortness + collisionFreeWeight * collision;
        if (fitness < this->bestFitness)
        {
            this->bestShortness = shortness;
            this->bestFitness = fitness;
            this->best = this->particles[i];
        }
    }
    if (this->bestFitness < this->greatestFitness)
    {
        SaveGreatest();
        this->greatestFitness = this->bestFitness;
        this->greatestShortness = this->bestShortness;
    }
    else
    {
        LoadGreatest();
    }
}

void PathOptimizer::CoefficientUpdate()
{
    this->a -= (this->_a / (long double)(this->_iterations * this->population));
    this->r = GenerateRandom(0.0l, 1.0l);
    this->A = (2.0l * this->a * this->r) - this->a;
    this->C = (2.0l * this->r);
    this->l = GenerateRandom(-1.0l, 1.0l);
    this->p = GenerateRandom(0.0l, 1.0l);
}

void PathOptimizer::CircleUpdate()
{
    for (int j = 1; j < this->size-1; ++j)
    {
        long double Dx = fabs((this->C * this->best.array[j].x) - this->particles[this->i].array[j].x);
        long double Dy = fabs((this->C * this->best.array[j].y) - this->particles[this->i].array[j].y);
        this->particles[this->i].array[j].x = this->best.array[j].x - (this->A * Dx);
        this->particles[this->i].array[j].y = this->best.array[j].y - (this->A * Dy);
    }
}

void PathOptimizer::RandomUpdate()
{
    int index = GenerateRandom(0, this->population);
    this->random = this->particles[index];
    for (int j = 1; j < this->size-1; ++j)
    {
        long double Dx = fabs((this->C * this->random.array[j].x) - this->particles[this->i].array[j].x);
        long double Dy = fabs((this->C * this->random.array[j].y) - this->particles[this->i].array[j].y);
        this->particles[this->i].array[j].x = this->random.array[j].x - (this->A * Dx);
        this->particles[this->i].array[j].y = this->random.array[j].y - (this->A * Dy);
    }
}

void PathOptimizer::SpiralUpdate()
{
    for (int j = 1; j < this->size-1; ++j)
    {
        long double Dx = fabs(this->best.array[j].x - this->particles[this->i].array[j].x);
        long double Dy = fabs(this->best.array[j].y - this->particles[this->i].array[j].y);
        this->particles[this->i].array[j].x = Dx * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].array[j].x;
        this->particles[this->i].array[j].y = Dy * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].array[j].y;
    }
}

void PathOptimizer::CheckBoundary()
{
    for (int i = 0; i < this->population; ++i)
    {
        for (int j = 1; j < this->size-1; ++j)
        {
            State& current = this->particles[i].array[j];
            bool isOutside = false;
            State* closestWaypoint;
            long double distance, shortestDistance = std::numeric_limits<long double>::max();
            for (int k = 1; k < this->size-1; ++k)
            {
                State& original = this->original.array[k]; 
                if (current.x > (original.x + this->widthBound))
                    isOutside = true;
                else if (current.x < (original.x - this->widthBound))
                    isOutside = true;
                if (current.y > (original.y + this->heightBound))
                    isOutside = true;
                else if (current.y < (original.y - this->heightBound))
                    isOutside = true;
                distance = CalculateEuclideanDistance(current, original);
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                    closestWaypoint = &original;
                }
            }
            if (isOutside)
            {
                current.x = closestWaypoint->x;
                current.y = closestWaypoint->y;
            }
            if (current.x < myMap.origin.x)
            {
                current.x = myMap.origin.x;
            }
            else if (current.x > -myMap.origin.x)
            {
                current.x = -myMap.origin.x;
            }
            if (current.y < myMap.origin.y)
            {
                current.y = myMap.origin.y;
            }
            else if (current.y > -myMap.origin.y)
            {
                current.y = -myMap.origin.y;
            }
        }
    }
}

void PathOptimizer::RenderParticles()
{
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    image = myMap.colored.clone();
    /// Draw all particles.
    int x, y;
    std::vector<Path> invalid; // Invalid paths.
    std::vector<Path> valid; // Valid paths.
    bool isValid;
    for (int i = 0; i < this->population; ++i)
    {
        isValid = true;
        for (int j = 1; j < this->size; ++j)
        {
            if (!IsObstacleFree(this->particles[i].array[j-1], this->particles[i].array[j]))
            {
                invalid.push_back(this->particles[i]);
                isValid = false;
                break;
            }
        }
        if (isValid){
            valid.push_back(this->particles[i]);
            continue;
        }
     
    }
    // Draw invalid paths.
    for (int i = 0; i < invalid.size(); ++i)
    {
        for (int j = 1; j < this->size; ++j)
        {
            x = XConvertToPixel(invalid[i].array[j-1].x);
            y = YConvertToPixel(invalid[i].array[j-1].y);
            cv::Point p(x, y);
            x = XConvertToPixel(invalid[i].array[j].x);
            y = YConvertToPixel(invalid[i].array[j].y);
            cv::Point q(x, y);
            line(image, p, q, cv::Scalar(0x00, 0x00, 0xFF), 1, cv::LINE_AA);
        }
    }
    // Draw valid paths.
    for (int i = 0; i < valid.size(); ++i)
    {
        for (int j = 1; j < this->size; ++j)
        {
            x = XConvertToPixel(valid[i].array[j-1].x);
            y = YConvertToPixel(valid[i].array[j-1].y);
            cv::Point p(x, y);
            x = XConvertToPixel(valid[i].array[j].x);
            y = YConvertToPixel(valid[i].array[j].y);
            cv::Point q(x, y);
            line(image, p, q, cv::Scalar(0xF4, 0x9D, 0x2A), 1, cv::LINE_AA);
        }
    }
    /// Draw currently best path.
    for (int j = 1; j < this->size; ++j)
    {
        x = XConvertToPixel(this->greatest.array[j-1].x);
        y = YConvertToPixel(this->greatest.array[j-1].y);
        cv::Point p(x, y);
        x = XConvertToPixel(this->greatest.array[j].x);
        y = YConvertToPixel(this->greatest.array[j].y);
        cv::Point q(x, y);
        line(image, p, q, cv::Scalar(0x00, 0xFF, 0x00), 3, cv::LINE_AA);
    }
    /// Goal configuration
    x = XConvertToPixel(this->best.array[this->size-1].x);
    y = YConvertToPixel(this->best.array[this->size-1].y);      
    cv::Point goal(x, y);
    circle(image, goal, 5, cv::Scalar(0x28, 0x81, 0xFA), -1, cv::LINE_AA);
    /// Init configuration
    x = XConvertToPixel(this->best.array[0].x);
    y = YConvertToPixel(this->best.array[0].y);
    cv::Point init(x, y);
    circle(image, init, 5, cv::Scalar(0xFF, 0x75, 0xC1), -1, cv::LINE_AA);
    bool renderGraphics = mySettings["RenderGraphics"];
    if (renderGraphics)
    {
        cv::imshow("RRT*WOA Path", image);
        cv::waitKey(0);
    }
    myVideo1->write(image);
    
}

void PathOptimizer::CleanUp()
{
    delete this->greatest.array;
    for (int i = 0; i < this->population; ++i)
    {
        delete this->particles[i].array;
    }
    delete this->particles;
}

/// ------------------------------------------------------------------------------------ 

long double PathOptimizer::GenerateRandom(long double a, long double b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution(a, b);
    return distribution(engine);
}

int PathOptimizer::GenerateRandom(int a, int b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution((long double)a, (long double)b);
    long double random = distribution(engine);
    return (int)(std::floor(random));  
}

long double PathOptimizer::CalculateEuclideanDistance(State& a, State& b)
{
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2));
}

int PathOptimizer::XConvertToPixel(long double x)
{
    return (int)((x + (-myMap.origin.x)) / myMap.resolution);
}

int PathOptimizer::YConvertToPixel(long double y)
{
    return (int)((y + (-myMap.origin.x)) / myMap.resolution);
}

bool PathOptimizer::DoIntersect(cv::Point& p1, cv::Point& q1, cv::Point& p2, cv::Point& q2)
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

int PathOptimizer::Orientation(cv::Point& p, cv::Point& q, cv::Point& r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool PathOptimizer::OnSegment(cv::Point& p, cv::Point& q, cv::Point& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;
    return false;
}

bool PathOptimizer::IsObstacleFree(State& sNew, State& sNeighbor)
{
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
            return false;
        }
    }
    return true;
}

void PathOptimizer::SaveGreatest()
{
    for (int i = 0; i < this->size; ++i)
    {
        this->greatest.array[i] = this->best.array[i];
    }
}

void PathOptimizer::LoadGreatest()
{
    for (int i = 0; i < this->size; ++i)
    {
        this->best.array[i] = this->greatest.array[i];
    }
}

Path PathOptimizer::ApplyV2()
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

Path PathOptimizer::ApplyV3()
{
    InitializePopulation();
    CalculateFitness();
    int search = 0.5 * this->_iterations;
    int attack = 0.5 * this->_iterations;
    this->iterations = search;
    this->_iterations = search;
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
    this->_iterations = attack;
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