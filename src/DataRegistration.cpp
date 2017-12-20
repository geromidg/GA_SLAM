#include "ga_slam/DataRegistration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

#include "grid_map_core/iterators/GridMapIterator.hpp"

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
    processCloud(cloud, sensorToMapTF);
    translateMap(estimatedPose.translation());
    updateMap();
}

void DataRegistration::processCloud(
        const Cloud::ConstPtr& cloud,
        const Pose& sensorToMapTF) {
    downsampleCloud(cloud);
    transformCloudToMap(sensorToMapTF);
    cropCloudToMap();
    calculateCloudVariances();
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

void DataRegistration::downsampleCloud(const Cloud::ConstPtr& cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
    voxelGrid.filter(*processedCloud_);
}

void DataRegistration::transformCloudToMap(const Pose& sensorToMapTF) {
    pcl::transformPointCloud(*processedCloud_, *processedCloud_, sensorToMapTF);
}

void DataRegistration::cropCloudToMap(void) {
    const auto& position = map_.getPosition();
    const auto pointX = (mapSizeX_ / 2) + position.x();
    const auto pointY = (mapSizeY_ / 2) + position.y();

    Eigen::Vector4f minCutoffPoint(-pointX, -pointY, minElevation_, 0.);
    Eigen::Vector4f maxCutoffPoint(pointX, pointY, maxElevation_, 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(processedCloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*processedCloud_);
}

void DataRegistration::calculateCloudVariances(void) {
    cloudVariances_.clear();
    cloudVariances_.reserve(processedCloud_->size());

    for (const auto& point : processedCloud_->points)
        cloudVariances_.push_back(1.);
}

void DataRegistration::fuseGaussians(
        float& mean1, float& variance1,
        const float& mean2, const float& variance2) {
    const double innovation = mean2 - mean1;
    const double gain = variance1 / (variance1 + variance2);

    mean1 = mean1 + (gain * innovation);
    variance1 = variance1 * (1. - gain);
}

void DataRegistration::convertMapToCloud(Cloud::Ptr& cloud) const {
    cloud->clear();
    cloud->reserve(map_.getSize().x() * map_.getSize().y());
    cloud->is_dense = true;
    cloud->header.stamp = map_.getTimestamp();

    const auto& meanData = map_.get(layerMeanZ_);
    grid_map::Position point;

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);

        map_.getPosition(index, point);
        cloud->push_back(pcl::PointXYZ(
                point.x(), point.y(), meanData(index(0), index(1))));
    }
}

double DataRegistration::measureCloudAlignment(
            const Cloud::ConstPtr& cloud1,
            const Cloud::ConstPtr& cloud2) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);

    Cloud transformedCloud;
    icp.align(transformedCloud);

    return icp.getFitnessScore();
}

}  // namespace ga_slam

