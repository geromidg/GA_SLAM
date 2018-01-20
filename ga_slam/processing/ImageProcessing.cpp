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
#include <cmath>

namespace ga_slam {

void ImageProcessing::convertMapToImage(const Map& map, Image& image) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();

    image = Image::zeros(params.size, params.size, CV_32F);

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const Eigen::Array2i mapIndex(*it);
        const float value = meanData(mapIndex(0), mapIndex(1));

        const Eigen::Array2i index(it.getUnwrappedIndex());
        if (std::isfinite(value)) image.at<float>(index(0), index(1)) = value;
    }
}

void ImageProcessing::displayImage(
        const Image& image,
        const std::string& windowName,
        double zoom) {
    displayImage(image, windowName,
            std::round(zoom * image.cols),
            std::round(zoom * image.rows));
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

bool ImageProcessing::findBestMatch(
        const Image& originalImage,
        const Image& templateImage,
        cv::Point& matchedPosition,
        double matchAcceptanceThreshold,
        bool useCrossCorrelation,
        bool displayMatch) {
    int method;
    Image resultImage;

    if (useCrossCorrelation)
        method = CV_TM_CCORR_NORMED;
    else
        method = CV_TM_CCOEFF_NORMED;

    cv::matchTemplate(originalImage, templateImage, resultImage, method);

    cv::Point maxPosition;
    double maxValue;
    cv::minMaxLoc(resultImage, nullptr, &maxValue, nullptr, &maxPosition);

    const bool matchFound = maxValue > matchAcceptanceThreshold;
    if (matchFound) matchedPosition = maxPosition;

    if (matchFound && displayMatch)
        displayMatchedPosition(originalImage, templateImage, resultImage,
                matchedPosition);

    return matchFound;
}

void ImageProcessing::displayMatchedPosition(
        const Image& originalImage,
        const Image& templateImage,
        const Image& resultImage,
        const cv::Point& matchedPosition,
        double zoom) {
    Image originalImageClone = originalImage.clone();
    Image templateImageClone = templateImage.clone();
    Image resultImageClone = resultImage.clone();

    const auto matchedDiagon = cv::Point(
            matchedPosition.x + templateImageClone.cols,
            matchedPosition.y + templateImageClone.rows);

    const auto color = cv::Scalar::all(0.);
    cv::rectangle(originalImageClone, matchedPosition, matchedDiagon, color);
    cv::rectangle(resultImageClone, matchedPosition, matchedDiagon, color);

    ImageProcessing::displayImage(originalImageClone, "Original Image", zoom);
    ImageProcessing::displayImage(templateImageClone, "Template Image", zoom);
    ImageProcessing::displayImage(resultImageClone, "Result Image", zoom);
}

}  // namespace ga_slam

