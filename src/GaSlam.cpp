#include "ga_slam/GaSlam.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

#include "grid_map_core/iterators/GridMapIterator.hpp"

namespace ga_slam {

GaSlam::GaSlam(const Map& globalMap)
        : globalMap_(globalMap),
          layerMeanZ_("meanZ"),
          layerVarZ_("varZ") {
    rawMap_ = Map({layerMeanZ_, layerVarZ_});
    rawMap_.setBasicLayers({layerMeanZ_, layerVarZ_});
    rawMap_.clearBasic();
    rawMap_.resetTimestamp();

    filteredCloud_.reset(new Cloud);
}

bool GaSlam::setParameters(
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

    rawMap_.setGeometry(grid_map::Length(mapSizeX_, mapSizeY_), mapResolution_,
            grid_map::Position(robotPositionX_, robotPositionY_));

    return true;
}

void GaSlam::registerData(
        const Cloud::ConstPtr& inputCloud,
        const Pose& inputPose,
        const Pose& sensorToBodyTF,
        const Pose& bodyToGroundTF) {
    auto bodyToMapTF = inputPose * bodyToGroundTF;
    const auto& sensorToMapTF = bodyToMapTF * sensorToBodyTF;

    preProcessCloud(inputCloud, sensorToMapTF);
    translateMap(bodyToMapTF.translation());
    updateMap();

    correctedPose_ = bodyToMapTF;
}

void GaSlam::fuseMap(void) {}

void GaSlam::correctPose(void) {}

void GaSlam::preProcessCloud(
        const Cloud::ConstPtr& inputCloud,
        const Pose& sensorToMapTF) {
    downsampleCloud(inputCloud);
    transformCloudToMap(sensorToMapTF);
    cropCloudToMap();
    calculateCloudVariances();
}

void GaSlam::translateMap(const Eigen::Vector3d& translation) {
    grid_map::Position positionXY(translation.x(), translation.y());

    rawMap_.move(positionXY);
}

void GaSlam::updateMap(void) {
    auto& meanData = rawMap_.get(layerMeanZ_);
    auto& varianceData = rawMap_.get(layerVarZ_);

    size_t cloudIndex = 0;
    grid_map::Index mapIndex;

    for (const auto& point : filteredCloud_->points) {
        cloudIndex++;

        if (!rawMap_.getIndex(grid_map::Position(point.x, point.y), mapIndex))
            continue;

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

    rawMap_.setTimestamp(filteredCloud_->header.stamp);
}

void GaSlam::downsampleCloud(const Cloud::ConstPtr& inputCloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
    voxelGrid.filter(*filteredCloud_);
}

void GaSlam::transformCloudToMap(const Pose& sensorToMapTF) {
    pcl::transformPointCloud(*filteredCloud_, *filteredCloud_, sensorToMapTF);
}

void GaSlam::cropCloudToMap(void) {
    const auto& position = rawMap_.getPosition();
    const auto pointX = (mapSizeX_ / 2) + position.x();
    const auto pointY = (mapSizeY_ / 2) + position.y();

    Eigen::Vector4f minCutoffPoint(-pointX, -pointY, minElevation_, 0.);
    Eigen::Vector4f maxCutoffPoint(pointX, pointY, maxElevation_, 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(filteredCloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*filteredCloud_);
}

void GaSlam::calculateCloudVariances(void) {
    cloudVariances_.clear();
    cloudVariances_.reserve(filteredCloud_->size());

    for (const auto& point : filteredCloud_->points)
        cloudVariances_.push_back(1.);
}

void GaSlam::fuseGaussians(
        float& mean1, float& variance1,
        const float& mean2, const float& variance2) {
    const double innovation = mean2 - mean1;
    const double gain = variance1 / (variance1 + variance2);

    mean1 = mean1 + (gain * innovation);
    variance1 = variance1 * (1. - gain);
}

void GaSlam::convertMapToCloud(Cloud::Ptr& cloud) const {
    cloud->clear();
    cloud->reserve(rawMap_.getSize().x() * rawMap_.getSize().y());
    cloud->is_dense = true;
    cloud->header.stamp = rawMap_.getTimestamp();

    const auto& meanData = rawMap_.get(layerMeanZ_);
    grid_map::Position point;

    for (grid_map::GridMapIterator it(rawMap_); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);

        rawMap_.getPosition(index, point);
        cloud->push_back(pcl::PointXYZ(
                point.x(), point.y(), meanData(index(0), index(1))));
    }
}

double GaSlam::measureCloudAlignment(
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

