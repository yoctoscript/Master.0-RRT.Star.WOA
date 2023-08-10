#include <opencv2/opencv.hpp>
#include "MapProcessor.hpp"
#include "Objects.hpp"
#include "Logger.hpp"
#include "Debug.hpp"

void MapProcessor::FindSegments(){
    #ifdef DEBUG
        static Logger log(__FUNCTION__);
    #endif
    /// Performs image thresholding on a grayscale image using a threshold value of 250 and creates a binary image.
    #ifdef DEBUG
        log.Debug("threshold()");
    #endif
    cv::Mat binary;
    threshold(this->image, binary, 250, 255, cv::THRESH_BINARY_INV);
    #ifdef DEBUG
        cv::imshow("Image", this->image);
        cv::imshow("Binary", binary);
    #endif
    /// Performs morphological dilation on a binary image using a 3x3 rectangular structuring element, repeated n times. 1:1px
    #ifdef DEBUG
        log.Debug("dilate()");
    #endif
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    for (int i = 0; i < this->dilation; ++i)
    {
        dilate(binary, binary, kernel);
    }
    #ifdef DEBUG
        cv::imshow("Dilated", binary);
    #endif
    /// Detects contours on a binary image using 'findContours()' function and stores the detected contours and their hierarchy in 'contours' and 'hierarchy' respectively.
    #ifdef DEBUG
        log.Debug("findContours()");
    #endif
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    #ifdef DEBUG
        log.Debug("Contoured Objects: {}", contours.size());
        cv::Mat invertedPolygonImage = cv::Mat::zeros(this->height, this->width, CV_8UC1);
        log.Debug("approxPolyDP()");
    #endif
    for (int i = 0; i < contours.size(); ++i)
    {
        /// Approximates a contour to a polygon using the 'approxPolyDP()' function with an epsilon value based on 'epsilon' times the contour's arc length. The result is stored in polygon.
        std::vector<cv::Point> polygon;
        double epsilon = this->epsilon * arcLength(contours[i], true);
        approxPolyDP(contours[i], polygon, epsilon, true);
        int polygonSize = polygon.size();
        /// Checks for a single point obstacle and creates a Segment object with its coordinates as start and end points. It then adds the Segment object to a vector of obstacle segments.
        if (polygonSize == 1)
        {
            cv::Point a = polygon[0];
            Segment seg(a, a);
            this->segments.push_back(seg);
            #ifdef DEBUG
                line(invertedPolygonImage, a, a, cv::Scalar(255), 1, cv::LINE_AA);
                log.Trace("Obstacle Point ({}, {})", a.x, a.y);
            #endif
        }
        /// Checks for a two-point obstacle and creates a Segment object with the two points as start and end points. It then adds the Segment object to a vector of obstacle segments.
        else if (polygonSize == 2)
        {
            cv::Point a = polygon[0];
            cv::Point b = polygon[1];
            Segment seg(a, b);
            this->segments.push_back(seg);
            #ifdef DEBUG
                line(invertedPolygonImage, a, b, cv::Scalar(255), 1, cv::LINE_AA);
                log.Trace("Obstacle Segment ({}, {}) ({}, {})", a.x, a.y, b.x, b.y);
            #endif
        }
        /// Checks if the polygon has more than 2 points. If so, it creates segments between each pair of adjacent points and adds them to a vector of obstacle segments.
        else if (polygonSize > 2)
        {
            for (int j = 0; j < polygonSize; ++j)
            {
                if (j == 0)
                {
                    cv::Point a = polygon[polygonSize-1];
                    cv::Point b = polygon[0];
                    Segment seg(a, b);
                    this->segments.push_back(seg);
                    #ifdef DEBUG
                        line(invertedPolygonImage, a, b, cv::Scalar(255), 1, cv::LINE_AA);
                        log.Trace("Obstacle Segment ({}, {}) ({}, {})", a.x, a.y, b.x, b.y);
                    #endif
                }
                else
                {
                    cv::Point a = polygon[j-1];
                    cv::Point b = polygon[j];
                    Segment seg(a, b);
                    this->segments.push_back(seg);
                    #ifdef DEBUG
                        line(invertedPolygonImage, a, b, cv::Scalar(255), 1, cv::LINE_AA);
                        log.Trace("Obstacle Segment ({}, {}) ({}, {})", a.x, a.y, b.x, b.y);
                    #endif
                }
            }
        }
    }
    #ifdef DEBUG
        /// Show the result.
        cv::Mat polygonImage;
        threshold(invertedPolygonImage, polygonImage, 128, 255, cv::THRESH_BINARY_INV);
        imshow("Polygon", polygonImage);
        log.Debug("Total Segments: {}", this->segments.size());
    #endif
}