#ifndef WOA_HPP
#define WOA_HPP
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "Objects.hpp"
#include "Logger.hpp"

extern nlohmann::json mySettings;

class PathOptimizer
{
    public:
        /// Constructors.
        PathOptimizer(Path& path)
        {
            this->original = path;
            this->size = path.size;
            this->_a = mySettings["PathOptimizer_a_Parameter"];
            this->a = this->_a;
            this->b = mySettings["PathOptimizer_b_Parameter"];
            this->population = mySettings["PathOptimizer_Population"];
            this->_iterations = mySettings["PathOptimizer_Iterations"];
            this->iterations = this->_iterations;
            this->heightBound = mySettings["PathOptimizer_Height_Bound"];
            this->widthBound = mySettings["PathOptimizer_Width_Bound"];
            this->pathShortnessWeight = mySettings["PathOptimizer_Shortness_Weight"];
            this->collisionFreeWeight = mySettings["PathOptimizer_Collision_Free_Weight"];
        }
        PathOptimizer(){}; /// Default constructor.
        /// Fields.
        Path original;
        int size;
        Path* particles;
        Path random;
        Path best;
        Path greatest;
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
        long double pathShortnessWeight;
        long double collisionFreeWeight;
        long double bestFitness;
        long double bestShortness;
        long double greatestFitness;
        long double greatestShortness;
        /// Methods.
        Path Apply(); /// Main function.
        Path ApplyV2();
        Path ApplyV3();
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