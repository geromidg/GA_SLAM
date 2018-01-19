#include "ga_slam/processing/ImageProcessing.hpp"

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/mapping/Map.hpp"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <string>

namespace ga_slam {

void ImageProcessing::convertMapToImage(const Map& map, Image& image) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();

    image = Image::zeros(params.size, params.size, CV_32F);

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const Eigen::Array2i index(*it);
        const Eigen::Array2i imageIndex(it.getUnwrappedIndex());
        const float value = meanData(index(0), index(1));
        image.at<float>(imageIndex(0), imageIndex(1)) = value;
    }
}

void ImageProcessing::displayImage(
        const Image& image,
        const std::string& windowName,
        int width,
        int height) {
    constexpr int waitTimeout = 100;

    Image normalizedImage;
    cv::normalize(image, normalizedImage, 0., 1., cv::NORM_MINMAX);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, width, height);
    cv::imshow(windowName, normalizedImage);
    cv::waitKey(waitTimeout);
}

void ImageProcessing::calculateGradientImage(
        const Image& inputImage,
        Image& outputImage,
        bool useSobel,
        int sobelKernelSize,
        bool approximate) {
    Image gradientX, gradientY;

    if (useSobel) {
        cv::Sobel(inputImage, gradientX, CV_32F, 1, 0, sobelKernelSize);
        cv::Sobel(inputImage, gradientY, CV_32F, 0, 1, sobelKernelSize);
    } else {
        cv::Scharr(inputImage, gradientX, CV_32F, 1, 0);
        cv::Scharr(inputImage, gradientY, CV_32F, 0, 1);
    }

    if (approximate) {
        cv::abs(gradientX);
        cv::abs(gradientY);
        cv::addWeighted(gradientX, 0.5, gradientY, 0.5, 0., outputImage);
    } else {
        cv::magnitude(gradientX, gradientY, outputImage);
    }
}

void ImageProcessing::calculateLaplacianImage(
        const Image& inputImage,
        Image& outputImage,
        int laplacianKernelSize,
        bool applyGaussianBlur,
        int gaussianKernelSize) {
    if (applyGaussianBlur) {
        cv::GaussianBlur(inputImage, outputImage,
                cv::Size(gaussianKernelSize, gaussianKernelSize), 0, 0);
        cv::Laplacian(outputImage, outputImage, CV_32F, laplacianKernelSize);
        cv::abs(outputImage);
    } else {
        cv::Laplacian(inputImage, outputImage, CV_32F, laplacianKernelSize);
    }
}

}  // namespace ga_slam

