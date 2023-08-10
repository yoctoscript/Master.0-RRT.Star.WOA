#ifndef PathPlannerHeader
#define PathPlannerHeader
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "Objects.hpp"

class PathPlanner
{
    public:
        /// Constructors.
        PathPlanner(State& sInit, State& sGoal)
        {
            this->sInit = sInit;
            this->sGoal = sGoal;
        }
        PathPlanner(){} /// Default constructor.
        /// Fields.
        State* sTree; /// States tree.
        State sInit; /// Initial state.
        State sGoal; /// Goal state.
        int iterations; /// Number of states to generate.
        long double searchRadius; /// Search radius.
        long double goalThreshold; /// Goal threshold.
        long double stepSize;
        int count; /// State counter.
        /// Methods.
        Path Build(); /// Tree generation.
        void Initialize(); /// Initialize.
        void InsertRoot(); /// Insert initial state.
        State SampleFreeSpace(); /// Sample free space.
        State* FindNearest(State& sRand); /// Find nearest.
        State Steer(State* sNear, State& sRand); /// Steer.
        bool IsObstacleFree(State& sNew); /// Is obstacle free (Point).
        bool IsObstacleFree(State& sNew, State& sNeighbor); /// Is obstacle free (Segment).
        std::vector<State*> GetNeighbors(State& sNew); /// Get neighbors.
        State* ChooseParent(State& sNew, std::vector<State*>& neighbors); /// Choose parent.
        State* Insert(State& sNew, State* sParent); /// Insert.
        void RewireTree(State* sNew, std::vector<State*>& neighbors); /// Rewire tree.
        Path ShortestPath(); /// Shortest path.
        void Render(Path& path); /// Visualization.
        void CleanUp(); /// Deallocation.
        /// Helper methods.
        long double CalculateEuclideanDistance(State&a, State& b); /// Calculate euclidean distance.
        bool DoIntersect(cv::Point& a, cv::Point& b, cv::Point& p, cv::Point& q); /// Verify the intersection of a segment with obstacles' segments.
        int Orientation(cv::Point& p, cv::Point& q, cv::Point& r); /// Returns the orientation of a triplet.
        bool OnSegment(cv::Point& p, cv::Point& q, cv::Point& r); /// Returns the collinearity of a triplet.
        int XConvertToPixel(long double x); /// Convert x-axis real coordinate to map coordinate.
        int YConvertToPixel(long double y); /// Convert y-axis real coordinate to map coordinate.
        long double GenerateRandom(long double min, long double max); /// Return a random number between [min, max].
};
#endif