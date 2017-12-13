#include "GaSlam.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace ga_slam {

GaSlam::GaSlam(const Map& globalMap)
        : globalMap_(globalMap),
          layerMeanZ_("meanZ"),
          layerVarX_("varX"),
          layerVarY_("varY"),
          layerVarZ_("varZ") {
    rawMap_ = Map({layerMeanZ_, layerVarX_, layerVarY_, layerVarZ_});
    rawMap_.setBasicLayers({layerMeanZ_});
    rawMap_.clearBasic();
    rawMap_.resetTimestamp();

    filteredCloud_.reset(new PointCloud);
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
        const PointCloud::ConstPtr& inputCloud,
        const Pose& inputPose,
        const Pose& sensorToBodyTF,
        const Pose& bodyToGroundTF) {
    auto bodyToMapTF = inputPose * bodyToGroundTF;
    const auto& sensorToMapTF = bodyToMapTF * sensorToBodyTF;

    processPointCloud(inputCloud, sensorToMapTF);
    translateMap(bodyToMapTF.translation());
    updateMap();

    correctedPose_ = bodyToMapTF;
}

void GaSlam::fuseMap(void) {}

void GaSlam::correctPose(void) {}

void GaSlam::processPointCloud(
        const PointCloud::ConstPtr& inputCloud,
        const Pose& sensorToMapTF) {
    downsamplePointCloud(inputCloud);
    transformPointCloudToMap(sensorToMapTF);
    cropPointCloudToMap();
}

void GaSlam::translateMap(const Eigen::Vector3d& translation) {
    grid_map::Position positionXY(translation.x(), translation.y());

    rawMap_.move(positionXY);
}

void GaSlam::updateMap(void) {
    const std::string layerNewMeanZ("newMeanZ");
    const std::string layerCount("count");
    rawMap_.add(layerNewMeanZ, NAN);
    rawMap_.add(layerCount, 0.);

    auto& meanZData = rawMap_.get(layerMeanZ_);
    auto& newMeanZData = rawMap_.get(layerNewMeanZ);
    auto& countData = rawMap_.get(layerCount);

    // Calculate measurement map (newMeanZ layer) from point cloud
    for (const auto& point : filteredCloud_->points) {
        grid_map::Index index;

        if (!rawMap_.getIndex(grid_map::Position(point.x, point.y), index))
            continue;

        float& newMeanZ = newMeanZData(index(0), index(1));
        float& count = countData(index(0), index(1));

        if (!count)
            newMeanZ = point.z;
        else
            newMeanZ = (newMeanZ * count + point.z) / (count + 1.);

        count++;
    }

    // Fuse measurement map with prior map
    for (grid_map::GridMapIterator it(rawMap_); !it.isPastEnd(); ++it) {
        const auto& index = it.getLinearIndex();

        float& meanZ = meanZData(index);
        float& newMeanZ = newMeanZData(index);

        if (!std::isfinite(newMeanZ))
            continue;

        if (!std::isfinite(meanZ))
            meanZ = newMeanZ;
        else
            meanZ = (meanZ + newMeanZ) / 2.;
    }

    rawMap_.setTimestamp(filteredCloud_->header.stamp);
}

void GaSlam::downsamplePointCloud(const PointCloud::ConstPtr& inputCloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
    voxelGrid.filter(*filteredCloud_);
}

void GaSlam::transformPointCloudToMap(const Pose& sensorToMapTF) {
    pcl::transformPointCloud(*filteredCloud_, *filteredCloud_, sensorToMapTF);
}

void GaSlam::cropPointCloudToMap(void) {
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

}  // namespace ga_slam

