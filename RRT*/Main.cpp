#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include "Objects.hpp"
#include "Logger.hpp"
#include "Debug.hpp"
#include "MapProcessor.hpp"
#include "PathPlanner.hpp"
#include "PathSmoother.hpp"
#include "ControlGenerator.hpp"

nlohmann::json mySettings; /// Object containing the contents of "settings.json".
MapProcessor myMap; /// Object containing map metadata.
cv::VideoWriter* myVideo; /// Object used to write videos.
void LoadSettings();

int main(int argc, char** argv){
    static Logger log(__FUNCTION__);
    LoadSettings();
    std::string mapName = "maps/";
    mapName += mySettings["MapProcessor_Map_Name"];
    mapName += ".pgm";
    /// Data provided by ROS.
    cv::Mat img = cv::imread(mapName, cv::IMREAD_GRAYSCALE);
    State sInit(nullptr, -11.0l, 11.0l, 3.14l, 0.0l, 0.0l, 0.0l, 0.0l);
    State sGoal(nullptr, 11.0l, -11.0l, 1.0l, 0.0l, 0.0l, 0.0l, 0.0l);
    long double x = -12.5l, y = -12.5l, resolution = 0.05l;
    int width = 500, height = 500;
    State origin(x, y);

    /// Filling the map object and retrieving obstacles.
    auto mStart = std::chrono::high_resolution_clock::now();
    myMap.image = img;
    myMap.origin = origin;
    myMap.resolution = resolution;
    myMap.width = img.cols;
    myMap.height = img.rows;
    myMap.dilation = mySettings["MapProcessor_Obstacles_Dilation"];
    myMap.epsilon = mySettings["MapProcessor_Epsilon"];
    myMap.FindSegments();
    auto mEnd = std::chrono::high_resolution_clock::now();
    auto mElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(mEnd - mStart);
    log.Info("MAP  => Processed in: {:.2f} milliseconds", (long double)mElapsedTime.count() / 1000.0l);

    bool testMode = mySettings["TestMode"];

    /// Building path.
    if (!testMode)
    {
        myVideo = new cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 1, cv::Size(500, 500), true);
        auto rStart = std::chrono::high_resolution_clock::now();
        PathPlanner rrt_star(sInit, sGoal);
        Path path = rrt_star.Build();
        auto rEnd = std::chrono::high_resolution_clock::now();
        auto rElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(rEnd - rStart);
        log.Info("RRT* => Built in: {:.2f} milliseconds", (long double)rElapsedTime.count() / 1000.0l);
        log.Info("Path length = {:.2f} meters", path.array[path.size-1].c);
        rrt_star.Render(path);
        /*
        Path newPath = SmoothenPath(path);
        PathControl pathCtrl(sInit, newPath, sGoal);
        pathCtrl.Generate();
        */
        rrt_star.CleanUp();
        delete path.array;
        myVideo->release();
        cv::waitKey();
        return 0;
    }
}

void LoadSettings(){
    std::ifstream f("Settings.json");
    mySettings = nlohmann::json::parse(f);
    f.close();
}