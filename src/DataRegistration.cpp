#include "ga_slam/DataRegistration.hpp"

namespace ga_slam {

void DataRegistration::setParameters(
        double mapLengthX, double mapLengthY, double mapResolution,
        double minElevation, double maxElevation) {
    map_.setMapParameters(mapLengthX, mapLengthY, mapResolution,
            minElevation, maxElevation);
}

void DataRegistration::translateMap(const Pose& estimatedPose) {
    map_.translate(estimatedPose.translation());
}

void DataRegistration::updateMap(
        const Cloud::ConstPtr& cloud,
        const std::vector<float>& cloudVariances) {
    auto& meanData = map_.getMeanZ();
    auto& varianceData = map_.getVarianceZ();

    size_t cloudIndex = 0;
    size_t mapIndex;

    for (const auto& point : cloud->points) {
        cloudIndex++;

        if (!map_.getIndexFromPosition(point.x, point.y, mapIndex)) continue;

        float& mean = meanData(mapIndex);
        float& variance = varianceData(mapIndex);
        const float& pointVariance = cloudVariances[cloudIndex - 1];

        if (!std::isfinite(mean)) {
            mean = point.z;
            variance = pointVariance;
        } else {
            fuseGaussians(mean, variance, point.z, pointVariance);
        }
    }

    map_.setValid(true);
    map_.setTimestamp(cloud->header.stamp);
}

void DataRegistration::fuseGaussians(
        float& mean1, float& variance1,
        const float& mean2, const float& variance2) {
    const double innovation = mean2 - mean1;
    const double gain = variance1 / (variance1 + variance2);

    mean1 = mean1 + (gain * innovation);
    variance1 = variance1 * (1. - gain);
}

}  // namespace ga_slam

