#ifndef MapProcessorHeader
#define MapProcessorHeader
#include <opencv2/opencv.hpp>
#include "Objects.hpp"

extern nlohmann::json mySettings;

class MapProcessor
{
    public:
        /// Constructors.
        MapProcessor(){} /// Default constructor.
        /// Fields.
        cv::Mat image; /// Matrix containing my map data in grayscale.
        cv::Mat colored; /// Matrix containing colored representation of the map.
        State origin; /// Origin of real world map, corresponds to (0.0)
        long double resolution; /// Meters / pixel.
        int width; /// Width of the map.
        int height; /// Height of the map.
        int dilation; /// Number of extra pixels added to obstacles contour.
        long double epsilon; /// The smaller the value, the more precision.
        Segments segments; /// A vector containing all obstacles segments.
        /// Methods.
        void FindSegments(); /// Method to retrieve obstacles' segments from image.
};
#endif