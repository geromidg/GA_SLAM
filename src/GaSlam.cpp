#include "GaSlam.hpp"

#include <pcl/common/transforms.h>

namespace ga_slam {

GaSlam::GaSlam(const Map& globalMap)
        : globalMap_(globalMap) {
    filteredCloud_.reset(new PointCloud);
}

void GaSlam::registerData(
        const Pose& pose,
        const Pose& tf,
        const PointCloud::ConstPtr& pointCloud) {
    downsamplePointCloud();
    transformPointCloudToMap(pose, tf, pointCloud);
    cropPointCloudToMap();
}

void GaSlam::fuseMap(void) {}

void GaSlam::correctPose(void) {}

void GaSlam::translateMap(const Pose& deltaPose) {}

void GaSlam::updateMap(const PointCloud::ConstPtr& pointCloud) {}

void GaSlam::downsamplePointCloud(void) {}

void GaSlam::transformPointCloudToMap(
        const Pose& pose,
        const Pose& tf,
        const PointCloud::ConstPtr& inputCloud) {
}

void GaSlam::cropPointCloudToMap(void) {}

}  // namespace ga_slam

