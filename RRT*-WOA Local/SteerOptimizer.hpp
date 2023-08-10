#ifndef WOA_HPP
#define WOA_HPP
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "Objects.hpp"
#include "Logger.hpp"
#include "Debug.hpp"

extern nlohmann::json mySettings;

class SteerOptimizer
{
    public:
        /// Constructors.
        SteerOptimizer(State near, State original, State goal)
        {
            this->near = near;
            this->original = original;
            this->goal = goal;
            this->_a = mySettings["SteerOptimizer_a_Parameter"];
            this->a = this->_a;
            this->b = mySettings["SteerOptimizer_b_Parameter"];
            this->population = mySettings["SteerOptimizer_Population"];
            this->_iterations = mySettings["SteerOptimizer_Iterations"];
            this->iterations = this->_iterations;
            this->heightBound = mySettings["SteerOptimizer_Height_Bound"];
            this->widthBound = mySettings["SteerOptimizer_Width_Bound"];
            this->distanceToGoalWeight = mySettings["SteerOptimizer_DistanceToGoal_Weight"];
            this->distanceToRandWeight = mySettings["SteerOptimizer_DistanceToRand_Weight"];
        }
        SteerOptimizer(){}; /// Default constructor.
        /// Fields.
        State* particles;
        State original;
        State near;
        State goal;
        State random;
        State best;
        State greatest;
        int i;
        int population;
        int _iterations; /// Constant
        int iterations;
        long double _a; /// Constant
        long double a;
        long double r;
        long double A;
        long double C;
        long double p;
        long double l;
        long double b;
        long double heightBound;
        long double widthBound;
        long double bestFitness;
        long double greatestFitness;
        long double distanceToGoalWeight;
        long double distanceToRandWeight;
        /// Methods.
        State Apply(); /// Main function.
        State ApplyV2();
        State ApplyV3();
        void InitializePopulation();
        void CalculateFitness();
        void CoefficientUpdate();
        void CircleUpdate();
        void SpiralUpdate();
        void RandomUpdate();
        void CheckBoundary();
        void RenderParticles();
        void CleanUp();

        /// Helper methods.
        long double GenerateRandom(long double a, long double b);
        int GenerateRandom(int a, int b);
        long double CalculateEuclideanDistance(State& a, State& b);
        int XConvertToPixel(long double x);
        int YConvertToPixel(long double y);
        bool IsObstacleFree(State& sNew, State& sNeighbor);
        bool DoIntersect(cv::Point& p1, cv::Point& q1, cv::Point& p2, cv::Point& q2);
        int Orientation(cv::Point& p, cv::Point& q, cv::Point& r);
        bool OnSegment(cv::Point& p, cv::Point& q, cv::Point& r);
        void SaveGreatest();
        void LoadGreatest();
};
#endif