#include "ControlGenerator.hpp"
#include "Objects.hpp"
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include "Logger.hpp"
#include "MapProcessor.hpp"
#include <iomanip>
#include <fstream>
#include <iostream>

extern nlohmann::json mySettings;
extern MapProcessor myMap;
extern cv::VideoWriter* myVideo1;

struct Obs{
    State s;
    double incX;
    double incY;
};

Obs movingObjects[5] { 
    State(475.0, 382.0), -0.3, 0.0, 
    State(24.0, 29.0), +2.0, 0.0,
    State(184.0, 71.0), 0.9, 0.9,
    State(37.0, 211.0), 0.5, 0.,
    State(459.0, 44.0), 0.0 , +1.2,
};

Path SegmentControl::Apply()
{
    InitializePopulation();
    CalculateFitness();
    while (this->iterations--)
    {
        for (this->i = 0; this->i < this->population; ++(this->i))
        {
            CoefficientUpdate();
            if ((this->p) < 0.5L)
            {
                if (fabs(this->A) < 0.5L)
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
        //RenderParticles();
    }
    CleanUp();
    return this->greatest;
}

void SegmentControl::InitializePopulation()
{
    this->particles = new Path[this->population];
    for (int i = 0; i < this->population; ++i)
    {
        particles[i].size = this->pathSize;
        particles[i].array = new State[this->pathSize];
        for (int j = 0; j < this->pathSize; ++j)
        {
            this->particles[i].array[j].v = GenerateRandom(this->minLinearVelocity, this->maxLinearVelocity);
            this->particles[i].array[j].w = GenerateRandom(this->minAngularVelocity, this->maxAngularVelocity);
            this->particles[i].array[j].t = GenerateRandom(this->minTimeStep, this->maxTimeStep);
        }
    }
    this->greatestFitness =  std::numeric_limits<long double>::max();
    this->greatest.size = this->pathSize;
    this->greatest.array = new State[this->pathSize];
}

void SegmentControl::CalculateFitness()
{
    this->bestFitness = std::numeric_limits<long double>::max();
    long double length, compatibility, fitness;
    double dx = (sGoal.x - sInit.x) / (this->pathSize + 1);
    double dy = (sGoal.y - sInit.y) / (this->pathSize + 1);
    for (int i = 0; i < this->population; ++i)
    {
        compatibility = length = 0.0l;
        long double x = this->sInit.x;
        long double y = this->sInit.y;
        long double z = this->sInit.z;
        for (int j = 0; j < this->pathSize; ++j)
        {
            long double oldX = x;
            long double oldY = y;
            long double v = this->particles[i].array[j].v;
            long double w = this->particles[i].array[j].w;
            long double t = this->particles[i].array[j].t;
            x = this->particles[i].array[j].x = x - (v*t) * sin(z + (w*t) / 2.0l);
            y = this->particles[i].array[j].y = y + (v*t) * cos(z + (w*t) / 2.0l);
            z = this->particles[i].array[j].z = z + (w*t);
            compatibility += sqrt(pow(x - (sInit.x + (j+1) * dx),2) + pow(y - (sInit.y + (j+1) * dy), 2));            
            length += CalculateEuclideanDistance(oldX, oldY, x ,y);
        }
        long double distanceToGoal = CalculateEuclideanDistance(x, y, sGoal.x, sGoal.y);
        fitness =   this->compatibilityWeight * compatibility +
                    this->lengthWeight * fabs(length - CalculateEuclideanDistance(sInit.x, sInit.y, sGoal.x, sGoal.y)) +
                    this->distanceToGoalWeight * distanceToGoal;
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

void SegmentControl::CoefficientUpdate()
{
    this->a -= (this->_a / (long double)(this->_iterations * this->population));
    this->r = GenerateRandom(0.0l, 1.0l);
    this->A = (2 * this->a * this->r) - this->a;
    this->C = (2 * this->r);
    this->l = GenerateRandom(-1.0l, 1.0l);
    this->p = GenerateRandom(0.0l, 1.0l);
}

void SegmentControl::CircleUpdate()
{
    for (int j = 0; j < this->pathSize; ++j)
    {
        long double Dv = fabs((this->C * this->best.array[j].v) - this->particles[this->i].array[j].v);
        long double Dw = fabs((this->C * this->best.array[j].w) - this->particles[this->i].array[j].w);
        long double Dt = fabs((this->C * this->best.array[j].t) - this->particles[this->i].array[j].t);
        this->particles[this->i].array[j].v = this->best.array[j].v - (this->A * Dv);
        this->particles[this->i].array[j].w = this->best.array[j].w - (this->A * Dw);
        this->particles[this->i].array[j].t = this->best.array[j].t - (this->A * Dt);
    }
}

void SegmentControl::RandomUpdate()
{
    int index = GenerateRandom(0, this->population);
    this->random = this->particles[index];
     for (int j = 0; j < this->pathSize; ++j)
    {
        long double Dv = fabs((this->C * this->random.array[j].v) - this->particles[this->i].array[j].v);
        long double Dw = fabs((this->C * this->random.array[j].w) - this->particles[this->i].array[j].w);
        long double Dt = fabs((this->C * this->random.array[j].t) - this->particles[this->i].array[j].t);
        this->particles[this->i].array[j].v = this->random.array[j].v - (this->A * Dv);
        this->particles[this->i].array[j].w = this->random.array[j].w - (this->A * Dw);
        this->particles[this->i].array[j].t = this->random.array[j].t - (this->A * Dt);
    }
}

void SegmentControl::SpiralUpdate()
{
    for (int j = 0; j < this->pathSize; ++j)
    {
        long double Dv = fabs(this->best.array[j].v - this->particles[this->i].array[j].v);
        long double Dw = fabs(this->best.array[j].w - this->particles[this->i].array[j].w);
        long double Dt = fabs(this->best.array[j].t - this->particles[this->i].array[j].t);
        this->particles[this->i].array[j].v = Dv * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].array[j].v;
        this->particles[this->i].array[j].w = Dw * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].array[j].w;
        this->particles[this->i].array[j].t = Dt * exp(b*l) * cos(2* M_PI * l) + this->particles[this->i].array[j].t;
    }
}

void SegmentControl::CheckBoundary()
{
    for (int i = 0; i < this->population; ++i)
    {
        for (int j = 0; j < this->pathSize; ++j)
        {
            if (this->particles[i].array[j].v > this->maxLinearVelocity)
            {
                this->particles[i].array[j].v = this->maxLinearVelocity;
            }
            else if (this->particles[i].array[j].v < this->minLinearVelocity)
            {
                this->particles[i].array[j].v = this->minLinearVelocity;
            }
            if (this->particles[i].array[j].w > this->maxAngularVelocity)
            {
                this->particles[i].array[j].w = this->maxAngularVelocity;
            }
            else if (this->particles[i].array[j].w < this->minAngularVelocity)
            {
                this->particles[i].array[j].w = this->minAngularVelocity;
            }
            if (this->particles[i].array[j].t > this->maxTimeStep)
            {
                this->particles[i].array[j].t = this->maxTimeStep;
            }
            else if (this->particles[i].array[j].t < this->minTimeStep)
            {
                this->particles[i].array[j].t = this->minTimeStep;
            }
        }
    }
}

void SegmentControl::RenderParticles()
{
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    /// Draw initial and goal coordinates.
    circle(image, cv::Point(XConvertToPixel(sInit.x), YConvertToPixel(sInit.y)), 5, cv::Scalar(0, 0xFF, 0), -1, cv::LINE_AA); 
    circle(image, cv::Point(XConvertToPixel(sGoal.x), YConvertToPixel(sGoal.y)), 5, cv::Scalar(0, 0xFF, 0), -1, cv::LINE_AA); 
    /// Draw all particles.
    int x, y;
    for (int i = 0; i < this->population; ++i){
        for (int j = 1; j < this->pathSize; ++j)
        {
            x = XConvertToPixel(this->particles[i].array[j-1].x);
            y = YConvertToPixel(this->particles[i].array[j-1].y);
            cv::Point p(x, y);              
            x = XConvertToPixel(this->particles[i].array[j].x);
            y = YConvertToPixel(this->particles[i].array[j].y);
            cv::Point q(x, y);
            line(image, p, q, cv::Scalar(0x0, 0x0, 0xFF), 1, cv::LINE_AA);
        }
    }
    /// Draw the greatest.
    for (int j = 1; j < this->best.size; ++j)
    {
        x = XConvertToPixel(this->best.array[j-1].x);
        y = YConvertToPixel(this->best.array[j-1].y);
        cv::Point p(x, y);
        x = XConvertToPixel(this->best.array[j].x);
        y = YConvertToPixel(this->best.array[j].y);
        cv::Point q(x, y);
        line(image, p, q, cv::Scalar(0x00, 0xFF, 0x00), 4, cv::LINE_AA);
    }
    cv::imshow("SegmentControl Particles", image);
    cv::waitKey(0);
}

void SegmentControl::CleanUp()
{
    ;
}

/// ------------------------------------------------------------------------------------ 

long double SegmentControl::GenerateRandom(long double a, long double b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution(a, b);
    return distribution(engine);
}

int SegmentControl::GenerateRandom(int a, int b)
{
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<long double> distribution((long double)a, (long double)b);
    long double random = distribution(engine);
    return (int)(std::floor(random));  
}

long double SegmentControl::CalculateEuclideanDistance(long double& x, long double& y, long double& a, long double& b){
    return sqrt(pow(x-a, 2)+pow(y-b, 2));
}

int SegmentControl::XConvertToPixel(long double& x)
{
    return (int)((x + (12.5l)) / 0.05l);
}

int SegmentControl::YConvertToPixel(long double& y)
{
    return (int)((y + (12.5l)) / 0.05l);
}

void SegmentControl::SaveGreatest()
{
    for (int i = 0; i < this->pathSize; ++i)
    {
        this->greatest.array[i] = this->best.array[i];
    }
}

void SegmentControl::LoadGreatest()
{
    for (int i = 0; i < this->pathSize; ++i)
    {
        this->best.array[i] = this->greatest.array[i];
    }
}







/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------
/// -----------------------------------------------------------------------------



extern cv::Mat image;




void PathControl::Generate()
{   
    /// Write to file
    std::ofstream outputFile("controls.txt");
    outputFile  << std::left << std::setw(15) << "Linear"
            << std::left << std::setw(15) << "Angular" 
            << std::left << std::setw(20) << "Duration" << std::endl;
    /// Display controls.
    cv::cvtColor(myMap.image, myMap.colored, cv::COLOR_GRAY2BGR);
    this->image = myMap.colored.clone();

    static Logger log(__FUNCTION__);
    SegmentControl segmentCtrl(this->sInit, this->path.array[0]);
    segmentCtrl.Apply();
    this->controls.push_back(segmentCtrl.greatest);
    ApplyControls(this->sInit, this->path.array[0], segmentCtrl.greatest);
    for (int i = 0; i < path.size - 1; ++i)
    {
        SegmentControl segmentCtrl(this->path.array[i], this->path.array[i+1]);
        segmentCtrl.Apply();
        for (int i = 0; i < segmentCtrl.pathSize; ++i)
        {
            if (fabs(segmentCtrl.greatest.array[i].t) <= std::numeric_limits<long double>::epsilon())
                continue;
            outputFile  << std::left << std::setw(15) << std::setprecision(2) << std::fixed << segmentCtrl.greatest.array[i].v 
                        << std::left << std::setw(15) << std::setprecision(2) << std::fixed << segmentCtrl.greatest.array[i].w 
                        << std::left << std::setw(20) << std::setprecision(10) << std::fixed << segmentCtrl.greatest.array[i].t << std::endl;
               }
        this->controls.push_back(segmentCtrl.greatest);
        ApplyControls(this->path.array[i], this->path.array[i+1], segmentCtrl.greatest);
    }
    cv::imshow("Controls", this->image);
    outputFile.close();

}

int XConvertToPixel(long double& x)
{
    return (int)((x + (12.5l)) / 0.05l);
}

int YConvertToPixel(long double& y)
{
    return (int)((y + (12.5l)) / 0.05l);
}


void PathControl::ApplyControls(State& start, State& end, Path controls)
{
    long double interval = mySettings["Simulation_Interval"];
    int X,Y;
    long double x = start.x;
    long double y = start.y;
    long double z = start.z;
    X = XConvertToPixel(x);
    Y = YConvertToPixel(y);      
    cv::Point a(X, Y);
    for (int j = 0; j < controls.size; ++j)
    {
        long double v = controls.array[j].v;
        long double w = controls.array[j].w;
        long double t = controls.array[j].t;
        if (fabs(t) <= std::numeric_limits<long double>::epsilon())
            continue;
        while (t >= interval)
        {
            if ((sqrt(pow(X-movingObjects[0].s.x,2)+pow(Y-movingObjects[0].s.y,2)) > 60) && 
            (sqrt(pow(X-movingObjects[1].s.x,2)+pow(Y-movingObjects[1].s.y,2)) > 60) &&
            (sqrt(pow(X-movingObjects[2].s.x,2)+pow(Y-movingObjects[2].s.y,2)) > 60) &&
            (sqrt(pow(X-movingObjects[3].s.x,2)+pow(Y-movingObjects[3].s.y,2)) > 60) &&
            (sqrt(pow(X-movingObjects[4].s.x,2)+pow(Y-movingObjects[4].s.y,2)) > 60))
            {
                x = x - (v*interval) * sin(z + (w*interval) / 2.0l);
                y = y + (v*interval) * cos(z + (w*interval) / 2.0l);
                z = z + (w*interval);
            }
            X = XConvertToPixel(x);
            Y = YConvertToPixel(y); 
            cv::Point b(X, Y);
            t-= interval;
            circle(this->image, b, 6, cv::Scalar(0xFF, 128, 128), 1, cv::LINE_AA); 
            cv::Mat temp = this->image.clone();
            circle(temp, b, 6, cv::Scalar(0xFF, 128, 0x00), -1, cv::LINE_AA);
            // orientation
            long double newx = x + 0.75 * cos(z + M_PI_2);
            long double newy = y + 0.75 * sin(z + M_PI_2);
            int newX = XConvertToPixel(newx);
            int newY = YConvertToPixel(newy);
            cv::Point end(newX, newY);
            line(temp, b, end, cv::Scalar(0xFF, 128, 0x00),2, cv::LINE_AA);
            {
                cv::Point m((int)movingObjects[0].s.x, (int)movingObjects[0].s.y);
                circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                movingObjects[0].s.x += movingObjects[0].incX; movingObjects[0].s.y += movingObjects[0].incY;
                if (movingObjects[0].s.x < 100 && movingObjects[0].incX < 0)
                {
                    movingObjects[0].incX = -movingObjects[0].incX;
                }
                if (movingObjects[0].s.x > 400 && movingObjects[0].incX > 0)
                {
                    movingObjects[0].incX = -movingObjects[0].incX;
                }
                m.x = (int)movingObjects[1].s.x; m.y = (int)movingObjects[1].s.y;
                circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                movingObjects[1].s.x += movingObjects[1].incX; movingObjects[1].s.y += movingObjects[1].incY;
                if (movingObjects[1].s.x < 100 && movingObjects[1].incX < 0)
                {
                    movingObjects[1].incX = -movingObjects[1].incX;
                }
                if (movingObjects[1].s.x > 400 && movingObjects[1].incX > 0)
                {
                    movingObjects[1].incX = -movingObjects[1].incX;
                }
                m.x = (int)movingObjects[2].s.x; m.y = (int)movingObjects[2].s.y;
                circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                movingObjects[2].s.x += movingObjects[2].incX; movingObjects[2].s.y += movingObjects[2].incY;
                if ((movingObjects[2].s.x < 100 && movingObjects[2].incX < 0) || (movingObjects[2].s.y < 100 && movingObjects[2].incY < 0) )
                {
                    movingObjects[2].incX = -movingObjects[2].incX;
                    movingObjects[2].incY = -movingObjects[2].incY;
                }
                if ((movingObjects[2].s.x > 400 && movingObjects[2].incX > 0) || (movingObjects[2].s.y > 400 && movingObjects[2].incY > 0))
                {
                    movingObjects[2].incX = -movingObjects[2].incX;
                     movingObjects[2].incY = -movingObjects[2].incY;
                  }
                m.x = (int)movingObjects[3].s.x; m.y = (int)movingObjects[3].s.y;
                circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                movingObjects[3].s.x += movingObjects[3].incX; movingObjects[3].s.y += movingObjects[3].incY;
                   if ((movingObjects[3].s.x < 100 && movingObjects[3].incX < 0) || (movingObjects[3].s.y < 100 && movingObjects[3].incY < 0) )
            {
                movingObjects[3].incX = -movingObjects[3].incX;
                movingObjects[3].incY = -movingObjects[3].incY;
            }
            if ((movingObjects[3].s.x > 400 && movingObjects[3].incX > 0) || (movingObjects[3].s.y > 400 && movingObjects[3].incY > 0))
            {
                movingObjects[3].incX = -movingObjects[3].incX;
                movingObjects[3].incY = -movingObjects[3].incY;
            }
                m.x = (int)movingObjects[4].s.x; m.y = (int)movingObjects[4].s.y;
                circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                movingObjects[4].s.x += movingObjects[4].incX; movingObjects[4].s.y += movingObjects[4].incY;
                    if (movingObjects[4].s.y < 100 && movingObjects[4].incY < 0)
            {
                movingObjects[4].incY = -movingObjects[4].incY;
            }
            if (movingObjects[4].s.y > 400 && movingObjects[4].incY > 0)
            {
                movingObjects[4].incY = -movingObjects[4].incY;
            }
            }
            myVideo1->write(temp);

        }
        x = x - (v*t) * sin(z + (w*t) / 2.0l);
        y = y + (v*t) * cos(z + (w*t) / 2.0l);
        z = z + (w*t); 
        X = XConvertToPixel(x);
        Y = YConvertToPixel(y);      
        cv::Point b(X, Y);
        circle(this->image, b, 6, cv::Scalar(0xFF, 128, 64), 1, cv::LINE_AA);
        cv::Mat temp = this->image.clone();
        circle(temp, b, 6, cv::Scalar(0xFF, 128, 0x00), -1, cv::LINE_AA); 
        // Orientation 
        long double newx = x + 0.75 * cos(z + M_PI_2);
        long double newy = y + 0.75 * sin(z + M_PI_2);
        int newX = XConvertToPixel(newx);
        int newY = YConvertToPixel(newy);
        cv::Point end(newX, newY);
        line(temp, b, end, cv::Scalar(0xFF, 128, 0x00),2, cv::LINE_AA);
        {
            cv::Point m((int)movingObjects[0].s.x, (int)movingObjects[0].s.y);
            circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            movingObjects[0].s.x += movingObjects[0].incX; movingObjects[0].s.y += movingObjects[0].incY;
            if (movingObjects[0].s.x < 100 && movingObjects[0].incX < 0)
            {
                movingObjects[0].incX = -movingObjects[0].incX;
            }
            if (movingObjects[0].s.x > 400 && movingObjects[0].incX > 0)
            {
                movingObjects[0].incX = -movingObjects[0].incX;
            }
            m.x = (int)movingObjects[1].s.x; m.y = (int)movingObjects[1].s.y;
            circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            movingObjects[1].s.x += movingObjects[1].incX; movingObjects[1].s.y += movingObjects[1].incY;
            if (movingObjects[1].s.x < 100 && movingObjects[1].incX < 0)
            {
                movingObjects[1].incX = -movingObjects[1].incX;
            }
            if (movingObjects[1].s.x > 400 && movingObjects[1].incX > 0)
            {
                movingObjects[1].incX = -movingObjects[1].incX;
            }
            m.x = (int)movingObjects[2].s.x; m.y = (int)movingObjects[2].s.y;
            circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            movingObjects[2].s.x += movingObjects[2].incX; movingObjects[2].s.y += movingObjects[2].incY;
            if ((movingObjects[2].s.x < 100 && movingObjects[2].incX < 0) || (movingObjects[2].s.y < 100 && movingObjects[2].incY < 0) )
            {
                movingObjects[2].incX = -movingObjects[2].incX;
                movingObjects[2].incY = -movingObjects[2].incY;
            }
            if ((movingObjects[2].s.x > 400 && movingObjects[2].incX > 0) || (movingObjects[2].s.y > 400 && movingObjects[2].incY > 0))
            {
                movingObjects[2].incX = -movingObjects[2].incX;
                movingObjects[2].incY = -movingObjects[2].incY;
            }
            m.x = (int)movingObjects[3].s.x; m.y = (int)movingObjects[3].s.y;
            circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            movingObjects[3].s.x += movingObjects[3].incX; movingObjects[3].s.y += movingObjects[3].incY;
              if ((movingObjects[3].s.x < 100 && movingObjects[3].incX < 0) || (movingObjects[3].s.y < 100 && movingObjects[3].incY < 0) )
            {
                movingObjects[3].incX = -movingObjects[3].incX;
                movingObjects[3].incY = -movingObjects[3].incY;
            }
            if ((movingObjects[3].s.x > 400 && movingObjects[3].incX > 0) || (movingObjects[3].s.y > 400 && movingObjects[3].incY > 0))
            {
                movingObjects[3].incX = -movingObjects[3].incX;
                movingObjects[3].incY = -movingObjects[3].incY;
            }
            m.x = (int)movingObjects[4].s.x; m.y = (int)movingObjects[4].s.y;
            circle(temp, m, 10, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            movingObjects[4].s.x += movingObjects[4].incX; movingObjects[4].s.y += movingObjects[4].incY;
            if (movingObjects[4].s.y < 100 && movingObjects[4].incY < 0)
            {
                movingObjects[4].incY = -movingObjects[4].incY;
            }
            if (movingObjects[4].s.y > 400 && movingObjects[4].incY > 0)
            {
                movingObjects[4].incY = -movingObjects[4].incY;
            }
        }
        myVideo1->write(temp);
    }
    end.x = x;
    end.y = y;
    end.z = z;
}




// void SegmentControl::CalculateFitness()
// {
//     this->bestFitness = std::numeric_limits<long double>::max();
//     long double length, compatibility, fitness;
//     double dx = (sGoal.x - sInit.x) / (this->pathSize + 1);
//     double dy = (sGoal.y - sInit.y) / (this->pathSize + 1);
//     for (int i = 0; i < this->population; ++i)
//     {
//         compatibility = length = 0.0l;
//         long double x = this->sInit.x;
//         long double y = this->sInit.y;
//         long double z = this->sInit.z;
//         for (int j = 0; j < this->pathSize; ++j)
//         {
//             long double oldX = x;
//             long double oldY = y;
//             long double v = this->particles[i].array[j].v;
//             long double w = this->particles[i].array[j].w;
//             long double t = this->particles[i].array[j].t;
//             x = this->particles[i].array[j].x = x - (v*t) * sin(z + (w*t) / 2.0l);
//             y = this->particles[i].array[j].y = y + (v*t) * cos(z + (w*t) / 2.0l);
//             z = this->particles[i].array[j].z = z + (w*t);
//             compatibility += sqrt(pow(x - (sInit.x + (j+1) * dx),2) + pow(y - (sInit.y + (j+1) * dy), 2));            
//             length += CalculateEuclideanDistance(oldX, oldY, x ,y);
//         }
//         long double distanceToGoal = CalculateEuclideanDistance(x, y, sGoal.x, sGoal.y);
//         fitness = this->compatibilityWeight * compatibility +
//                 this->lengthWeight * length +
//                 this->distanceToGoalWeight * distanceToGoal;
//         if (fitness < this->bestFitness)
//         {
//             this->bestFitness = fitness;
//             this->best = this->particles[i];
//         }
//     }
//     if (this->bestFitness < this->greatestFitness)
//     {
//         SaveGreatest();
//         this->greatestFitness = this->bestFitness;
//     }
//     else
//     {
//         LoadGreatest();
//     }
// }
