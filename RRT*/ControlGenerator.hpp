#ifndef ControlGenerator_HPP
#define ControlGenerator_HPP
#include "Objects.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

extern nlohmann::json mySettings;

class SegmentControl
{
    public:
        SegmentControl(State sInit, State sGoal)
        {
            this->sInit = sInit;
            this->sGoal = sGoal;
            this->_a = mySettings["ControlGenerator_a"];
            this->a = this->_a;
            this->b = mySettings["ControlGenerator_b"];
            this->population = mySettings["ControlGenerator_Population"];
            this->_iterations = mySettings["ControlGenerator_Iterations"];
            this->iterations = this->_iterations;
            this->maxLinearVelocity = mySettings["ControlGenerator_MaxLinearVelocity"];
            this->minLinearVelocity = mySettings["ControlGenerator_MinLinearVelocity"];
            this->maxAngularVelocity = mySettings["ControlGenerator_MaxAngularVelocity"];
            this->minAngularVelocity = mySettings["ControlGenerator_MinAngularVelocity"];
            this->maxTimeStep = mySettings["ControlGenerator_MaxTimeStep"];
            this->minTimeStep = mySettings["ControlGenerator_MinTimeStep"];
            this->pathSize = mySettings["ControlGenerator_PathSize"];
            this->lengthWeight = mySettings["ControlGenerator_LengthWeight"];
            this->compatibilityWeight = mySettings["ControlGenerator_CompatibilityWeight"];
            this->distanceToGoalWeight = mySettings["ControlGenerator_DistanceToGoalWeight"];
        }
        /// Fields.
        State sInit;
        State sGoal;
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
        long double maxLinearVelocity;
        long double minLinearVelocity;
        long double maxAngularVelocity;
        long double minAngularVelocity;
        long double maxTimeStep;
        long double minTimeStep;
        long double bestFitness;
        long double greatestFitness;
        long double lengthWeight;
        long double compatibilityWeight;
        long double distanceToGoalWeight;
        int pathSize;
        /// Methods.
        Path Apply(); /// Main function.
        void InitializePopulation();
        void CalculateFitness();
        void CoefficientUpdate();
        void CircleUpdate();
        void SpiralUpdate();
        void RandomUpdate();
        void CheckBoundary();
        void RenderParticles();
        void CleanUp();
        void SaveGreatest();
        void LoadGreatest();
        /// Helper methods.
        long double GenerateRandom(long double a, long double b);
        int GenerateRandom(int a, int b);
        long double CalculateEuclideanDistance(long double& x, long double& y, long double& a, long double& b);
        long double CalculateDistance(long double& v);
        long double CalculateAngle(long double& w);
        int XConvertToPixel(long double& x);
        int YConvertToPixel(long double& y);
};

class PathControl
{
    public:
        PathControl(State& sInit, Path path, State& sGoal)
        {
            this->sInit = sInit;
            this->path = path;
            this->sGoal = sGoal;
        }
        cv::Mat image;
        State sInit;
        Path path;
        State sGoal;
        std::vector<Path> controls;
        void Generate();
        void ApplyControls(State& start, State& end, Path controls);
};
#endif