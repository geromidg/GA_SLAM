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
        const Pose& inputPose,
        const Pose& cameraToMapTF,
        const PointCloud::ConstPtr& inputCloud) {
    transformMap(inputPose);
    updateMap(inputPose, cameraToMapTF, inputCloud);
}

void GaSlam::fuseMap(void) {}

void GaSlam::correctPose(void) {}

void GaSlam::transformMap(const Pose& inputPose) {}

void GaSlam::updateMap(
        const Pose& inputPose,
        const Pose& cameraToMapTF,
        const PointCloud::ConstPtr& inputCloud) {
    downsamplePointCloud(inputCloud);
    transformPointCloudToMap(inputPose, cameraToMapTF);
    cropPointCloudToMap();

    rawMap_.clearBasic();

    for (const auto& point : filteredCloud_->points) {
        grid_map::Index index;

        if (!rawMap_.getIndex(grid_map::Position(point.x, point.y), index))
            continue;

        auto& meanZ = rawMap_.at(layerMeanZ_, index);

        if (rawMap_.isValid(index, layerMeanZ_) && meanZ > point.z)
            continue;

        meanZ = point.z;
    }

    rawMap_.setTimestamp(inputCloud->header.stamp);
}

void GaSlam::downsamplePointCloud(const PointCloud::ConstPtr& inputCloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
    voxelGrid.filter(*filteredCloud_);
}

void GaSlam::transformPointCloudToMap(
        const Pose& pose,
        const Pose& cameraToMapTF) {
    Pose poseRotation, tfWithPose;
    double roll, pitch, yaw;

    pcl::getEulerAngles(pose, roll, pitch, yaw);
    pcl::getTransformation(0., 0., 0., -roll, -pitch, 0., poseRotation);
    tfWithPose = poseRotation * cameraToMapTF;

    pcl::transformPointCloud(*filteredCloud_, *filteredCloud_, tfWithPose);
}

void GaSlam::cropPointCloudToMap(void) {
    pcl::CropBox<pcl::PointXYZ> cropBox;

    Eigen::Vector4f minCutoffPoint(
            -(mapSizeX_ / 2) - robotPositionX_,
            -(mapSizeY_ / 2) - robotPositionY_,
            minElevation_,
            0.);

    Eigen::Vector4f maxCutoffPoint(
            (mapSizeX_ / 2) - robotPositionX_,
            (mapSizeY_ / 2) - robotPositionY_,
            maxElevation_,
            0.);

    cropBox.setInputCloud(filteredCloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*filteredCloud_);
}

}  // namespace ga_slam

