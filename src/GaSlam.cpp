#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_() {}

void GaSlam::setParameters(
        double mapSizeX, double mapSizeY,
        double robotPositionX, double robotPositionY,
        double mapResolution, double voxelSize,
        double minElevation, double maxElevation) {
    dataRegistration_.setParameters(mapSizeX, mapSizeY,
            robotPositionX, robotPositionY, mapResolution, voxelSize,
            minElevation, maxElevation);
}

void GaSlam::cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToBodyTF,
            const Pose& bodyToGroundTF,
            const Pose& poseGuess) {
    poseEstimation_.estimatePose(poseGuess * bodyToGroundTF);

    const auto& estimatedPose = poseEstimation_.getPose();
    const auto& sensorToMapTF = estimatedPose * sensorToBodyTF;
    dataRegistration_.registerData(cloud, sensorToMapTF, estimatedPose);
}

}  // namespace ga_slam

