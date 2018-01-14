#include "ga_slam/ImageProcessing.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ga_slam {

cv::Mat ImageProcessing::convertMapToImage(const Map& map) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();
    cv::Mat image = cv::Mat::zeros(params.sizeX, params.sizeY, CV_32F);

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        const grid_map::Index imageIndex(it.getUnwrappedIndex());
        const float value = meanData(index(0), index(1));
        image.at<float>(imageIndex(0), imageIndex(1)) = value;
    }

    return image;
}

void ImageProcessing::displayImage(
        const cv::Mat& image,
        const std::string& windowName,
        int width,
        int height) {
    constexpr int waitTimeout = 100;

    cv::Mat normalizedImage;
    cv::normalize(image, normalizedImage, 0., 1., cv::NORM_MINMAX);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, width, height);
    cv::imshow(windowName, normalizedImage);
    cv::waitKey(waitTimeout);
}

cv::Mat ImageProcessing::calculateGradientImage(
        const cv::Mat& image,
        bool useSobel,
        int sobelKernelSize,
        bool approximate) {
    cv::Mat gradient, gradientX, gradientY;

    if (useSobel) {
        cv::Sobel(image, gradientX, CV_32F, 1, 0, sobelKernelSize);
        cv::Sobel(image, gradientY, CV_32F, 0, 1, sobelKernelSize);
    } else {
        cv::Scharr(image, gradientX, CV_32F, 1, 0);
        cv::Scharr(image, gradientY, CV_32F, 0, 1);
    }

    if (approximate) {
        cv::abs(gradientX);
        cv::abs(gradientY);
        cv::addWeighted(gradientX, 0.5, gradientY, 0.5, 0., gradient);
    } else {
        cv::magnitude(gradientX, gradientY, gradient);
    }

    return gradient;
}

cv::Mat ImageProcessing::calculateLaplacianImage(
        const cv::Mat& image,
        int laplacianKernelSize,
        bool applyGaussianBlur,
        int gaussianKernelSize) {
    cv::Mat laplacian;

    if (applyGaussianBlur) {
        cv::GaussianBlur(image, laplacian,
                cv::Size(gaussianKernelSize, gaussianKernelSize), 0, 0);
        cv::Laplacian(laplacian, laplacian, CV_32F, laplacianKernelSize);
        cv::abs(laplacian);
    } else {
        cv::Laplacian(image, laplacian, CV_32F, laplacianKernelSize);
    }

    return laplacian;
}

}  // namespace ga_slam

