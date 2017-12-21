#include "ga_slam/DataRegistration.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

DataRegistration::DataRegistration(void)
        : layerMeanZ_("meanZ"),
          layerVarZ_("varZ") {
    map_ = Map({layerMeanZ_, layerVarZ_});
    map_.setBasicLayers({layerMeanZ_, layerVarZ_});
    map_.clearBasic();
    map_.resetTimestamp();

    processedCloud_.reset(new Cloud);
}

bool DataRegistration::setParameters(
        double mapSizeX, double mapSizeY,
        double robotPositionX, double robotPositionY,
        double mapResolution, double voxelSize,
        double minElevation, double maxElevation) {
    mapSizeX_ = mapSizeX;
    mapSizeY_ = mapSizeY;
    robotPositionX_ = robotPositionX;
    robotPositionY_ = robotPositionY;
    mapResolution_ = mapResolution;
    voxelSize_ = voxelSize;
    minElevation_ = minElevation;
    maxElevation_ = maxElevation;

    map_.setGeometry(grid_map::Length(mapSizeX_, mapSizeY_), mapResolution_,
            grid_map::Position(robotPositionX_, robotPositionY_));

    return true;
}

void DataRegistration::registerData(
        const Cloud::ConstPtr& cloud,
        const Pose& sensorToMapTF,
        const Pose& estimatedPose) {
    CloudProcessing::processCloud(cloud, processedCloud_, cloudVariances_,
            sensorToMapTF, map_, voxelSize_, minElevation_, maxElevation_);

    translateMap(estimatedPose.translation());
    updateMap();
}

void DataRegistration::translateMap(const Eigen::Vector3d& translation) {
    grid_map::Position positionXY(translation.x(), translation.y());

    map_.move(positionXY);
}

void DataRegistration::updateMap(void) {
    auto& meanData = map_.get(layerMeanZ_);
    auto& varianceData = map_.get(layerVarZ_);

    size_t cloudIndex = 0;
    grid_map::Index mapIndex;

    for (const auto& point : processedCloud_->points) {
        cloudIndex++;

        grid_map::Position position(point.x, point.y);
        if (!map_.getIndex(position, mapIndex)) continue;

        float& mean = meanData(mapIndex(0), mapIndex(1));
        float& variance = varianceData(mapIndex(0), mapIndex(1));
        const float& pointVariance = cloudVariances_[cloudIndex - 1];

        if (!std::isfinite(mean)) {
            mean = point.z;
            variance = pointVariance;
        } else {
            fuseGaussians(mean, variance, point.z, pointVariance);
        }
    }

    map_.setTimestamp(processedCloud_->header.stamp);
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

