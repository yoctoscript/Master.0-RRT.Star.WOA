#ifndef Smoother_HPP
#define Smoother_HPP
#include "Objects.hpp"
#include <vector>
#include "Logger.hpp"

extern nlohmann::json mySettings;
extern MapProcessor myMap;

/// Calculates the angle between two connected segments.
long double CalculateAngle(State p1, State p2, State p3)
{
    long double dx1 = p2.x - p1.x;
    long double dy1 = p2.y - p1.y;
    long double dx2 = p3.x - p2.x;
    long double dy2 = p3.y - p2.y;
    long double dotProduct = dx1 * dx2 + dy1 * dy2;
    long double magnitude1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    long double magnitude2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    long double cosTheta = dotProduct / (magnitude1 * magnitude2);
    long double angleRadians = std::acos(cosTheta);
    return angleRadians;
}

/// Returns true if all segments of the approximation respect the maximum turning angle, false if not.
bool RespectsTurningAngle(std::vector<State> segments)
{
    long double maxTurningAngle = mySettings["PathSmoother_MaxTurningAngle"];
    for (int i = 0; i < segments.size() - 2; ++i)
    {
        if (CalculateAngle(segments[i], segments[i+1], segments[i+2]) > maxTurningAngle)
        {
            return false;
        }
    }
    return true;
}

/// Returns mid-point between two given points.
State FindMidPoint(State p, State q)
{
    return State((p.x+q.x)/2.0l, (p.y+q.y)/2.0l);
}

Path SmoothenPath(Path _path)
{
    /// Removing duplicate states.
    std::vector<State> path;
    for (int i = 0; i < _path.size; ++i)
    { 
        if (i == 0)
        {
            path.push_back(_path.array[i]);
            continue;
        }
        else
        {
            bool condition1 = fabs(path.back().x -_path.array[i].x) <= std::numeric_limits<long double>::epsilon();
            bool condition2 = fabs(path.back().y -_path.array[i].y) <= std::numeric_limits<long double>::epsilon();
            if (condition1 && condition2)
            {
                continue;
            }
            else
            {
                path.push_back(_path.array[i]);
            }
        }
    }
    std::vector<State> smoothPath;
    int l;
    State actualStart, actualEnd;
    for (int i = 0; i < path.size(); i = i + l)
    {
        std::vector<State> segments;
        /// Initialize the first approximation with two segments.
        if (smoothPath.empty())
        {
            l = 3;
            actualStart = path[i];
            actualEnd = path[i+2];
            State p1 = FindMidPoint(path[i], path[i+1]);
            State p2 = path[i+1];
            State p3 = FindMidPoint(path[i+1],path[i+2]);
            segments.push_back(p1);
            segments.push_back(p2);
            segments.push_back(p3);
        }
        /// Initialize the subsequent approximations with the last segment + third point.
        else
        {
            l = 1;
            actualEnd = path[i];
            State p1 = smoothPath[smoothPath.size()-2];
            State p2 = smoothPath[smoothPath.size()-1];
            State p3 = FindMidPoint(smoothPath[smoothPath.size()-1], path[i]);
            smoothPath.pop_back();
            smoothPath.pop_back();
            segments.push_back(p1);
            segments.push_back(p2);
            segments.push_back(p3);
        }
        // While approximation doesn't respect maximum turning angle.
        while (!RespectsTurningAngle(segments))
        {
            std::vector<State> updated {segments.front()};
            for (int j = 1; j < segments.size(); ++j)
            {
                State mid = FindMidPoint(segments[j-1], segments[j]);
                updated.push_back(mid);
            }
            updated.push_back(segments.back());
            segments = updated;
        }
        /// If the vector is empty, push the actual the starting coordinates.
        if (smoothPath.empty())
        {
            smoothPath.push_back(actualStart);
        }
        /// Insert the approximation of mid-points.
        smoothPath.insert(smoothPath.end(), segments.begin(), segments.end());
        /// Insert the actual ending coordinates.
        smoothPath.push_back(actualEnd);
    }
    /// Display the smoothened path.
    cv::cvtColor(myMap.image, myMap.colored, cv::COLOR_GRAY2BGR);
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    image = myMap.colored.clone();
    int X,Y;
    for (int i = 1; i < path.size(); ++i)
    {
        X = (int)((path[i-1].x+12.5l)/0.05l);
        Y = (int)((path[i-1].y+12.5l)/0.05l);
        cv::Point p(X,Y);
        X = (int)((path[i].x+12.5l)/0.05l);
        Y = (int)((path[i].y+12.5l)/0.05l);
        cv::Point q(X,Y);
        line(image, p, q, cv::Scalar(0x0, 0xFF, 0x0), 1, cv::LINE_AA);
    }
    cv::imshow("Smoothened Path", image);
    /// Generate the <Path> object.
    Path pth;
    pth.size = smoothPath.size();
    pth.array = new State[pth.size];
    for (int i = 0; i < smoothPath.size(); ++i)
    {
        pth.array[i] = smoothPath[i];
    }
    return pth;
}
#endif