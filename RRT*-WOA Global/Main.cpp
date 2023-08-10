#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include "Objects.hpp"
#include "Logger.hpp"
#include "MapProcessor.hpp"
#include "PathPlanner.hpp"
#include "PathOptimizer.hpp"
#include "ControlGenerator.hpp"
#include "PathSmoother.hpp"
nlohmann::json mySettings; /// Object containing the contents of "settings.json".
MapProcessor myMap; /// Object containing map metadata.
cv::VideoWriter* myVideo; /// Object used to write videos.
cv::VideoWriter* myVideo1;
State movingObject(999,999);

State gsInit;
State gsGoal;
bool enough = false;
void LoadSettings();

int main(int argc, char** argv){
    // phase 1
    static Logger log(__FUNCTION__);
    LoadSettings();
    std::string mapName = "maps/";
    mapName += mySettings["MapProcessor_Map_Name"];
    mapName += ".pgm";
    /// Data provided by ROS.
    cv::Mat image = cv::imread("maps/map1.pgm", cv::IMREAD_GRAYSCALE);
    gsInit.x = -11.0l;
    gsInit.y = 11.0l;
    gsInit.z = 3.14l;
    gsGoal.x = 11.0l;
    gsGoal.y = -11.0l;
    gsGoal.z = 1.0l;
    long double x = -12.5l, y = -12.5l, resolution = 0.05l;
    int width = 500, height = 500;
    State origin(x, y);

    /// Filling the map object and retrieving obstacles.
    auto mStart = std::chrono::high_resolution_clock::now();
    myMap.image = image;
    myMap.origin = origin;
    myMap.resolution = resolution;
    myMap.width = image.cols;
    myMap.height = image.rows;
    myMap.dilation = mySettings["MapProcessor_Obstacles_Dilation"];
    myMap.epsilon = mySettings["MapProcessor_Epsilon"];
    myMap.FindSegments();
    auto mEnd = std::chrono::high_resolution_clock::now();
    auto mElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(mEnd - mStart);
    log.Info("MAP  => Processed in: {:.2f} milliseconds", (long double)mElapsedTime.count() / 1000.0l);

    myVideo = new cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(500, 500), true);
    myVideo1 = new cv::VideoWriter("output1.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(500, 500), true);
    auto pStart = std::chrono::high_resolution_clock::now();
    PathPlanner rrt_star(gsInit, gsGoal);
    Path path = rrt_star.Build();
    auto pEnd = std::chrono::high_resolution_clock::now();
    auto pElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(pEnd - pStart);
    log.Info("Path Planner => Built in: {:.3f} seconds", (long double)pElapsedTime.count() / 1000000.0l);
    log.Debug("Before Optimization > Path length = {:.2f} meters", path.array[path.size-1].c);
    //rrt_star.Render(path);
    
    auto oStart = std::chrono::high_resolution_clock::now();
    PathOptimizer woa(path);
    Path pth = woa.Apply();
    auto oEnd = std::chrono::high_resolution_clock::now();
    auto oElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(oEnd - oStart);
    log.Info("Path Optimizer => Built in: {:.3f} seconds", (long double)oElapsedTime.count() / 1000000.0l);
    log.Debug("After Optimization > Path length = {:.2f} meters", woa.greatestShortness);

    auto sStart = std::chrono::high_resolution_clock::now();
    Path newPath = SmoothenPath(pth);  
    auto sEnd = std::chrono::high_resolution_clock::now();
    auto sElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sEnd - sStart);
    log.Info("Path Smoother => Built in: {:.3f} seconds", (long double)sElapsedTime.count() / 1000000.0l);

    auto cStart = std::chrono::high_resolution_clock::now();
    PathControl pathCtrl(gsInit, newPath, gsGoal);
    pathCtrl.Generate();
    auto cEnd = std::chrono::high_resolution_clock::now();
    auto cElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(cEnd - cStart);
    log.Info("Path Control => Built in: {:.3f} seconds", (long double)cElapsedTime.count() / 1000000.0l);
    delete path.array;


    /// path 2
    enough = false;
    cv::Mat image2 = cv::imread("maps/map2.pgm", cv::IMREAD_GRAYSCALE);
    myMap.image = image2;
    myMap.FindSegments();
    PathPlanner rrt_star2(gsInit, gsGoal);
    Path path2 = rrt_star2.Build();
    PathOptimizer woa2(path2);
    Path pth2 = woa2.Apply();
    Path newPath2 = SmoothenPath(pth2);  
    PathControl pathCtrl2(gsInit, newPath2, gsGoal);
    pathCtrl2.Generate();

    // path 3

    enough = false;
    cv::Mat image3 = cv::imread("maps/map3.pgm", cv::IMREAD_GRAYSCALE);
    myMap.image = image3;
    myMap.FindSegments();
    PathPlanner rrt_star3(gsInit, gsGoal);
    Path path3 = rrt_star3.Build();
    PathOptimizer woa3(path3);
    Path pth3 = woa3.Apply();
    Path newPath3 = SmoothenPath(pth3);  
    PathControl pathCtrl3(gsInit, newPath3, gsGoal);
    pathCtrl3.Generate();















    myVideo->release();
    myVideo1->release();
    cv::waitKey();
    return 0;

}

void LoadSettings(){
    std::ifstream f("Settings.json");
    mySettings = nlohmann::json::parse(f);
    f.close();
}
