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
    Pose poseRotation, tfWithPose;
    double roll, pitch, yaw;

    pcl::getEulerAngles(pose, roll, pitch, yaw);
    pcl::getTransformation(0., 0., 0., -roll, -pitch, 0., poseRotation);
    tfWithPose = poseRotation * tf;

    pcl::transformPointCloud(*inputCloud, *filteredCloud_, tfWithPose);
}

void GaSlam::cropPointCloudToMap(void) {}

}  // namespace ga_slam

