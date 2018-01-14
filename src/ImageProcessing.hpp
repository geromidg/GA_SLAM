#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <opencv2/core/core.hpp>

namespace ga_slam {

class ImageProcessing {
  public:
    ImageProcessing(void) = delete;

    static cv::Mat convertMapToImage(const Map& map);

    static void displayImage(
            const cv::Mat& image,
            const std::string& windowName = "Image Display",
            int width = 500,
            int height = 500);

    static cv::Mat calculateGradientImage(
            const cv::Mat& image,
            bool useSobel = true,
            int sobelKernelSize = 3,
            bool approximate = false);

    static cv::Mat calculateLaplacianImage(
            const cv::Mat& image,
            int laplacianKernelSize = 1,
            bool applyGaussianBlur = false,
            int gaussianKernelSize = 3);
};

}  // namespace ga_slam

