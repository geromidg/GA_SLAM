#include "GaSlam.hpp"

namespace ga_slam {

GaSlam::GaSlam(const Map& globalMap)
        : globalMap_(globalMap) {
}

void GaSlam::registerData(
        const Pose& pose,
        const PointCloudConstPtr& pointCloud) {}

void GaSlam::translateMap(const Pose& deltaPose) {}

void GaSlam::updateMap(const PointCloudConstPtr& pointCloud) {}

void GaSlam::fuseMap(void) {}

void GaSlam::correctPose(void) {}

}  // namespace ga_slam

