#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

// OpenCV
#include <opencv2/core/core.hpp>

// STL
#include <string>

namespace ga_slam {

class ImageProcessing {
  public:
    ImageProcessing(void) = delete;

    static void convertMapToImage(const Map& map, Image& image);

    static void displayImage(
            const Image& image,
            const std::string& windowName = "Image Display",
            double zoom = 1.);

    static void displayImage(
            const Image& image,
            const std::string& windowName,
            int width,
            int height);

    static void calculateGradientImage(
            const Image& inputImage,
            Image& outputImage,
            bool useSobel = true,
            int sobelKernelSize = 3,
            bool approximate = false);

    static void calculateLaplacianImage(
            const Image& inputImage,
            Image& outputImage,
            int laplacianKernelSize = 1,
            bool applyGaussianBlur = false,
            int gaussianKernelSize = 3);

    static bool findBestMatch(
            const Image& originalImage,
            const Image& templateImage,
            cv::Point2d& matchedPosition,
            double matchAcceptanceThreshold,
            bool matchImageGradients = true,
            bool useCrossCorrelation = false,
            bool displayMatch = true);

    static void displayMatchedPosition(
            const Image& originalImage,
            const Image& templateImage,
            const Image& resultImage,
            const cv::Point2d& matchedPosition,
            double zoom = 4.);

    static void convertPositionToMapCoordinates(
        cv::Point2d& imagePosition,
        const Image& image,
        double mapResolution);
};

}  // namespace ga_slam

