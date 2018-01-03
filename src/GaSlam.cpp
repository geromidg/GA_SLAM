#include "ga_slam/GaSlam.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_() {
    processedCloud_.reset(new Cloud);
}

void GaSlam::setParameters(
        double mapLengthX, double mapLengthY, double mapResolution,
        double minElevation, double maxElevation,
        double voxelSize, int numParticles,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    voxelSize_ = voxelSize;

    poseEstimation_.setParameters(numParticles, initialSigmaX, initialSigmaY,
            initialSigmaYaw, predictSigmaX, predictSigmaY, predictSigmaYaw);

    dataRegistration_.setParameters(mapLengthX, mapLengthY, mapResolution,
            minElevation, maxElevation);
}

void GaSlam::poseCallback(const Pose& poseGuess, const Pose& bodyToGroundTF) {
    const auto transformedPoseGuess = poseGuess * bodyToGroundTF;

    poseEstimation_.predictPose(transformedPoseGuess);

    dataRegistration_.translateMap(poseEstimation_.getPose());
}

void GaSlam::cloudCallback(
        const Cloud::ConstPtr& cloud,
        const Pose& sensorToBodyTF) {
    std::vector<float> cloudVariances;
    const auto sensorToMapTF = poseEstimation_.getPose() * sensorToBodyTF;

    CloudProcessing::processCloud(cloud, processedCloud_, cloudVariances,
            sensorToMapTF, dataRegistration_.getMap(), voxelSize_);

    poseEstimation_.filterPose(dataRegistration_.getMap(), processedCloud_);

    dataRegistration_.updateMap(processedCloud_, cloudVariances);
}

}  // namespace ga_slam

