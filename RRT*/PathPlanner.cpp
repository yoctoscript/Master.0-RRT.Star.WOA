#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <inttypes.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <time.h>
#include <random>
#include <limits>
#include <cmath>
#include "Debug.hpp"
#include "Objects.hpp"
#include "Logger.hpp"
#include "MapProcessor.hpp"
#include "PathPlanner.hpp"

extern cv::VideoWriter* myVideo; /// Object used to write videos.
extern nlohmann::json mySettings;
extern MapProcessor myMap;

Path PathPlanner::Build(){
    static Logger log(__FUNCTION__);
    bool returnOnFirstPath = mySettings["ReturnOnFirstPath"];
    Initialize();
    InsertRoot();
    for (this->count = 1; this->count < this->iterations; ++(this->count))
    {               
        State sRand = SampleFreeSpace();
        State* sNear = FindNearest(sRand);
        State sNew = Steer(sNear, sRand);
        if (IsObstacleFree(sNew))
        {
            std::vector<State*> neighbors = GetNeighbors(sNew);
            if (neighbors.empty())
            {
                --(this->count);
                continue;
            }
            State* sParent = ChooseParent(sNew, neighbors);
            State* psNew = Insert(sNew, sParent);
            RewireTree(psNew, neighbors);
        }
        else
        {
            --(this->count);
            continue;
        }
        if (returnOnFirstPath)
        {
            if (CalculateEuclideanDistance(this->sTree[this->count], this->sGoal) <= this->goalThreshold)
            {
                ++this->count;
                log.Info("RRT* => Found path after: {} generated states.", this->count);
                return ShortestPath();
            }
        }
    }
    return ShortestPath();
}

void PathPlanner::Initialize()
{
    this->iterations = mySettings["PathPlanner_Iterations"];
    this->stepSize = mySettings["PathPlanner_Step_Size"];
    this->searchRadius = mySettings["PathPlanner_Search_Radius"];
    this->goalThreshold = mySettings["PathPlanner_Goal_Threshold"];
    this->sTree = new State[this->iterations];
    cv::cvtColor(myMap.image, myMap.colored, cv::COLOR_GRAY2BGR);
    return;
}

void PathPlanner::InsertRoot()
{
    sTree[0] = this->sInit;

    int x, y;
    /// Initial configuration.
    x = XConvertToPixel(this->sInit.x);
    y = YConvertToPixel(this->sInit.y);
    cv::Point init(x, y);
    circle(myMap.colored, init, 10, cv::Scalar(0xFF, 0x75, 0xC1), -1, cv::LINE_AA);

    /// Goal threshold region.
    x = XConvertToPixel(this->sGoal.x);
    y = YConvertToPixel(this->sGoal.y);
    cv::Point goal(x, y);
    int goalRadius = (int)(this->goalThreshold / myMap.resolution);
    circle(myMap.colored, goal, goalRadius, cv::Scalar(0x28, 0x81, 0xFA), -1, cv::LINE_AA);
    
    bool createVideo = mySettings["CreateVideo"];
    if (createVideo)
    {
        myVideo->write(myMap.colored);
    }
    return;
}

State PathPlanner::SampleFreeSpace()
{
    #ifdef DEBUG
        static Logger log(__FUNCTION__);
    #endif
    State sRand;
    sRand.x = GenerateRandom(myMap.origin.x, -myMap.origin.x);
    sRand.y = GenerateRandom(myMap.origin.y, -myMap.origin.y);
    #ifdef DEBUG
        log.Trace("sRand (x:{:.2f}, y:{:.2f})", sRand.x, sRand.y);
    #endif
    return sRand;
}

State* PathPlanner::FindNearest(State& sRand)
{
    #ifdef DEBUG
        static Logger log(__FUNCTION__);
    #endif
    State *sNear;
    long double shortestDistance = std::numeric_limits<long double>::max();
    for(int i = 0; i < this->count; ++i)
    {
        long double distance = CalculateEuclideanDistance(sRand, this->sTree[i]);
        if (distance < shortestDistance)
        {
            shortestDistance = distance;
            sNear = &(this->sTree[i]);
        }
    }
    #ifdef DEBUG
        log.Trace("sNear (x:{:.2f}, y:{:.2f}, z:{:.2f})", sNear->x, sNear->y, sNear->z);
    #endif
    return sNear;
}

State PathPlanner::Steer(State* sNear, State& sRand)
{
    double dx = sRand.x - sNear->x;
    double dy = sRand.y - sNear->y;
    double mag = std::sqrt(pow(dx, 2.0l) + pow(dy, 2.0l));
    double unitX = dx / mag;
    double unitY = dy / mag;
    double x = sNear->x + this->stepSize * unitX;
    double y = sNear->y + this->stepSize * unitY;
    if (x < myMap.origin.x)
    {
        x = myMap.origin.x;
    } 
    else if (x > -myMap.origin.x)
    {
        x = -myMap.origin.x;
    }
    if (y < myMap.origin.y)
    {
        y = myMap.origin.y;
    }
    else if (y > -myMap.origin.y)
    {
        y = -myMap.origin.y;
    }
    State sNew(x, y);
    return sNew;
}

bool PathPlanner::IsObstacleFree(State& sNew)
{
    #ifdef DEBUG
        static Logger log("IsObstacleFree [Point]");
    #endif
    int x = XConvertToPixel(sNew.x);
    int y = YConvertToPixel(sNew.y);
    cv::Point p(x, y);
    for (auto& segment: myMap.segments)
    {
        if (DoIntersect(p, p, segment.p, segment.q))
        {
            #ifdef DEBUG
                log.Trace("State (x:{:.2f}, y:{:.2f}) is not free", sNew.x, sNew.y);
            #endif
            return false;
        }
    }
    #ifdef DEBUG
        log.Trace("State (x:{:.2f}, y:{:.2f}) is free", sNew.x, sNew.y);
    #endif
    return true;
}

bool PathPlanner::IsObstacleFree(State& sNew, State& sNeighbor)
{
    #ifdef DEBUG
        static Logger log("IsObstacleFree [Segment]");
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
            return false;
        }
    }
    #ifdef DEBUG
        log.Trace("Segment p(x:{:.2f}, y:{:.2f}) q(x:{:.2f}, y:{:.2f}) is free", sNew.x, sNew.y, sNeighbor.x, sNeighbor.y);
    #endif
    return true;
}

std::vector<State*> PathPlanner::GetNeighbors(State& sNew)
{
    #ifdef DEBUG
        static Logger log(__FUNCTION__);
    #endif
    std::vector<State*> neighbors;
    for (int i = 0; i < this->count; ++i)
    {
        long double distance = CalculateEuclideanDistance(sNew, this->sTree[i]);
        if (distance < this->searchRadius)
        {
            if (IsObstacleFree(sNew, this->sTree[i]))
            {
                neighbors.push_back(&(this->sTree[i]));
            }
        }
    }
    return neighbors;
}

State* PathPlanner::ChooseParent(State& sNew, std::vector<State*>& neighbors)
{
    State* sParent;
    long double lowestCost = std::numeric_limits<long double>::max();
    for (auto& neighbor: neighbors)
    {
        long double cost = neighbor->c + CalculateEuclideanDistance(sNew, *neighbor);
        if (cost < lowestCost)
        {
            lowestCost = cost;
            sParent = neighbor;
        }
    }
    return sParent;
}

State* PathPlanner::Insert(State& sNew, State* sParent)
{
    long double cost = CalculateEuclideanDistance(sNew, *sParent);
    sNew.p = sParent;
    sNew.c = sParent->c + cost;
    this->sTree[this->count] = sNew;
   // circle(myMap.colored, cv::Point(XConvertToPixel(sNew.x), YConvertToPixel(sNew.y)), 5, cv::Scalar(0x00, 0x00, 0x00), -1, cv::LINE_AA);
    int x,y;
    State a = sNew;
    State b = *(a.p);
    x = XConvertToPixel(a.x);
    y = YConvertToPixel(a.y);
    cv::Point p(x, y); 
    circle(myMap.colored, p, 3, cv::Scalar(0x0, 0x0, 0x0), -1, cv::LINE_AA);
    x = XConvertToPixel(b.x);
    y = YConvertToPixel(b.y);
    cv::Point q(x, y);
    circle(myMap.colored, q, 3, cv::Scalar(0x0, 0x0, 0x0), -1, cv::LINE_AA);
    line(myMap.colored, p, q, cv::Scalar(0x0, 0x0, 0x0), 1, cv::LINE_AA);
    bool renderGraphics = mySettings["RenderGraphics"];
    if (renderGraphics)
    {
        cv::imshow("RRT*", myMap.colored);
        cv::waitKey(0);
    }
    bool createVideo = mySettings["CreateVideo"];
    if (createVideo)
    {
        myVideo->write(myMap.colored);
    }
    return &(this->sTree[this->count]);
}

void PathPlanner::RewireTree(State* sNew, std::vector<State*>& neighbors)
{
    State* parent = sNew->p;
    neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), parent), neighbors.end());
    for (auto& neighbor: neighbors)
    {
        double cost = sNew->c + CalculateEuclideanDistance(*sNew, *neighbor);
        if (cost < neighbor->c)
        {
            State* temp  = sNew;
            for (int i = 0; i < 3; ++i)
            {
                if (temp)
                {
                    temp = temp->p;
                }
            }
            if (temp == sNew) // If cycle happened.
            {
                continue;
            }
            neighbor->p = sNew;
            neighbor->c = cost;
        }
    }
    return;
}

void PathPlanner::Render(Path& path)
{
    cv::cvtColor(myMap.image, myMap.colored, cv::COLOR_GRAY2BGR);
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    image = myMap.colored.clone();

 
    int x, y;


    /// Initial configuration.
    x = XConvertToPixel(this->sInit.x);
    y = YConvertToPixel(this->sInit.y);
    cv::Point init(x, y);
    circle(image, init, 10, cv::Scalar(0xFF, 0x75, 0xC1), -1, cv::LINE_AA);

    /// Goal threshold region.
    x = XConvertToPixel(this->sGoal.x);
    y = YConvertToPixel(this->sGoal.y);
    cv::Point goal(x, y);
    int goalRadius = (int)(this->goalThreshold / myMap.resolution);
    circle(image, goal, goalRadius, cv::Scalar(0x28, 0x81, 0xFA), -1, cv::LINE_AA);
/*
    /// Draw all branches.
    for (int i = 1; i < this->count; ++i)
    {
        State a = this->sTree[i];
        State b = *(a.p);
        x = XConvertToPixel(a.x);
        y = YConvertToPixel(a.y);
        cv::Point p(x, y); 
        circle(image, p, 3, cv::Scalar(0x0, 0x0, 0x0), -1, cv::LINE_AA);
        x = XConvertToPixel(b.x);
        y = YConvertToPixel(b.y);
        cv::Point q(x, y);
        circle(image, q, 3, cv::Scalar(0x0, 0x0, 0x0), -1, cv::LINE_AA);
        line(image, p, q, cv::Scalar(0x0, 0x0, 0x0), 1, cv::LINE_AA);
    }

    

    /// Draw obstacle segments.
 //   for (auto& segment: myMap.segments)
   // {
     //   line(myMap.colored, segment.p, segment.q, cv::Scalar(0x2B, 0x24, 0xE3), 1, cv::LINE_AA);
   // }
    /// Draw shortest path.
    for (int i = 1; i < path.size; ++i)
    {
        State a = path.array[i];
        State b = *(a.p);
        x = XConvertToPixel(a.x);
        y = YConvertToPixel(a.y);
        cv::Point p(x, y); 
        x = XConvertToPixel(b.x);
        y = YConvertToPixel(b.y);
        cv::Point q(x, y); 
        line(image, p, q, cv::Scalar(0x00, 0xFF, 0x00), 3, cv::LINE_AA);
    }
*/
    cv::imshow("RRT* Path", image);
    cv::imwrite("map.png", image);
}

Path PathPlanner::ShortestPath(){
    #ifdef DEBUG
        static Logger log(__FUNCTION__);
    #endif
    //
   

    //
    double lowestCost = std::numeric_limits<long double>::max();
    State* bestState = nullptr;
    /// Find the state that lies within goal region and has the lowest cost.
    for (int i = 0; i < this->iterations; ++i)
    {
        long double distanceFromGoal = CalculateEuclideanDistance(this->sTree[i], this->sGoal);
        if (distanceFromGoal < this->goalThreshold)
        {
            if (this->sTree[i].c < lowestCost)
            {
                lowestCost = this->sTree[i].c;
                bestState = &(this->sTree[i]);
            }
        }
    }
    /// If no path found, return.
    if (!bestState)
    {
        #ifdef DEBUG
            log.Error("No path found .. exiting.");
        #endif
        exit(-1);
    }
    #ifdef DEBUG
        log.Trace("Best State: (x:{:.2f}, y:{:.2f}, z:{:.2f}, c:{:.2f})",bestState->x, bestState->y, bestState->z, bestState->c);
    #endif
    /// Find how many state constitute the shortest path.
    int i = 1;
    State* pointer = bestState;
    while (pointer->p)
    {
        ++i;
        pointer = pointer->p;
    }
    /// Copy the states into the path array.
    pointer = bestState;
    Path path;
    path.size = i;
    path.array = new State[i];
    for (int j = i-1; j >= 0; --j)
    {
        path.array[j] = *pointer;
        pointer = pointer->p;
    }


    // video
     int x,y;
    for (int i = 1; i < path.size; ++i)
    {
        State a = path.array[i];
        State b = *(a.p);
        x = XConvertToPixel(a.x);
        y = YConvertToPixel(a.y);
        cv::Point p(x, y); 
        x = XConvertToPixel(b.x);
        y = YConvertToPixel(b.y);
        cv::Point q(x, y); 
        line(myMap.colored, p, q, cv::Scalar(0x00, 0xFF, 0x00), 3, cv::LINE_AA);
       
    }
    bool createVideo = mySettings["CreateVideo"];
    if (createVideo)
    {
        for (int i = 0; i < 10; ++i)
            myVideo->write(myMap.colored);
    }



    return path;
}

void PathPlanner::CleanUp(){
    delete this->sTree;
    return;
}





//**********************************************************************
// This is utility section
//**********************************************************************        




long double PathPlanner::CalculateEuclideanDistance(State& a, State& b){
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2));
}

int PathPlanner::Orientation(cv::Point& p, cv::Point& q, cv::Point& r){
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool PathPlanner::OnSegment(cv::Point& p, cv::Point& q, cv::Point& r){
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;
    return false;
}

bool PathPlanner::DoIntersect(cv::Point& p1, cv::Point& q1, cv::Point& p2, cv::Point& q2){
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

int PathPlanner::XConvertToPixel(long double x){
    return (int)((x + (-myMap.origin.x)) / myMap.resolution);
}

int PathPlanner::YConvertToPixel(long double y){
    return (int)((y + (-myMap.origin.y)) / myMap.resolution);
}

long double PathPlanner::GenerateRandom(long double min, long double max){
    long double lowerBound = min;
    long double upperBound = max;
    std::default_random_engine re{std::random_device{}()};
    std::uniform_real_distribution<long double> unif(lowerBound, upperBound);
    return unif(re);
}

