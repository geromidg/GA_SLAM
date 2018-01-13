#include "ga_slam/PoseCorrection.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ga_slam {

void PoseCorrection::setParameters(
        double traversedDistanceThreshold,
        double slopeSumThreshold) {
    traversedDistanceThreshold_ = traversedDistanceThreshold;
    slopeSumThreshold_ = slopeSumThreshold;
}

void PoseCorrection::createGlobalMap(const Cloud::ConstPtr& cloud) {
    std::lock_guard<std::mutex> guard(globalMapMutex_);

    globalMap_.setParameters(100., 100., 1., -1000., 1000.);

    auto& meanData = globalMap_.getMeanZ();
    auto& varianceData = globalMap_.getVarianceZ();

    size_t cloudIndex = 0;
    size_t mapIndex;

    for (const auto& point : cloud->points) {
        cloudIndex++;

        if (!globalMap_.getIndexFromPosition(point.x, point.y, mapIndex))
            continue;

        float& mean = meanData(mapIndex);
        float& variance = varianceData(mapIndex);
        const float& pointVariance = 1.;

        if (!std::isfinite(mean)) {
            mean = point.z;
            variance = pointVariance;
        } else {
            const double innovation = point.z - mean;
            const double gain = variance / (variance + pointVariance);
            mean = mean + (gain * innovation);
            variance = variance * (1. - gain);
        }
    }

    globalMap_.setValid(true);
    globalMap_.setTimestamp(cloud->header.stamp);
}

bool PoseCorrection::distanceCriterionFulfilled(const Pose& pose) const {
    const Eigen::Vector3d currentXYZ = pose.translation();
    const Eigen::Vector3d lastXYZ = lastCorrectedPose_.translation();
    const Eigen::Vector2d deltaXY = currentXYZ.head(2) - lastXYZ.head(2);

    return deltaXY.norm() > traversedDistanceThreshold_;
}

bool PoseCorrection::featureCriterionFulfilled(const Map& localMap) const {

    return true;
}

cv::Mat PoseCorrection::calculateGradientMagnitudeImage(const cv::Mat& image) {
    cv::Mat gradient, gradientX, gradientY;

    cv::Sobel(image, gradientX, CV_32FC1, 1, 0, 3);
    cv::Sobel(image, gradientY, CV_32FC1, 0, 1, 3);

    cv::magnitude(gradientX, gradientY, gradient);

    return gradient;
}

cv::Mat PoseCorrection::calculateApproximateGradientMagnitudeImage(
        const cv::Mat& image) {
    cv::Mat gradient, gradientX, gradientY;

    cv::Sobel(image, gradientX, CV_32FC1, 1, 0, 3);
    cv::Sobel(image, gradientY, CV_32FC1, 0, 1, 3);

    cv::abs(gradientX);
    cv::abs(gradientY);
    cv::addWeighted(gradientX, 0.5, gradientY, 0.5, 0., gradient);

    return gradient;
}

cv::Mat PoseCorrection::calculateLaplacianImage(
        const cv::Mat& image) {
    cv::Mat laplacian;

    cv::Laplacian(image, laplacian, CV_32FC1, 3);

    return laplacian;
}

cv::Mat PoseCorrection::convertMapToImage(const Map& map) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();
    cv::Mat image = cv::Mat::zeros(params.sizeX, params.sizeY, CV_32FC1);

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        const grid_map::Index imageIndex(it.getUnwrappedIndex());
        const float value = meanData(index(0), index(1));
        image.at<float>(imageIndex(0), imageIndex(1)) = value;
    }

    return image;
}

void PoseCorrection::displayImage(
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

Pose PoseCorrection::matchMaps(const Pose& pose, const Map& localMap) {
    auto correctedPose = pose;

    lastCorrectedPose_= correctedPose;

    return correctedPose;
}

}  // namespace ga_slam

