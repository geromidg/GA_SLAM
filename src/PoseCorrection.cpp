#include "ga_slam/PoseCorrection.hpp"

namespace ga_slam {

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
    return true;
}

bool PoseCorrection::featureCriterionFulfilled(const Map& map) const {
    return true;
}

Pose PoseCorrection::matchMaps(const Pose& pose, const Map& map) {
    auto correctedPose = pose;

    lastCorrectedPose_= correctedPose;

    return correctedPose;
}

}  // namespace ga_slam

