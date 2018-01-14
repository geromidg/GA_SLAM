#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <opencv2/core/core.hpp>

namespace ga_slam {

class ImageProcessing {
  public:
    ImageProcessing(void) = delete;

    static void convertMapToImage(const Map& map, Image& image);

    static void displayImage(
            const Image& image,
            const std::string& windowName = "Image Display",
            int width = 500,
            int height = 500);

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
};

}  // namespace ga_slam

