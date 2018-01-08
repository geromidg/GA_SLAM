#include "ga_slam/GaSlam.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_(),
          poseInitialized_(false) {
}

void GaSlam::setParameters(
        double mapLengthX, double mapLengthY, double mapResolution,
        double minElevation, double maxElevation, double voxelSize,
        int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    voxelSize_ = voxelSize;

    poseEstimation_.setParameters(numParticles, resampleFrequency,
            initialSigmaX, initialSigmaY, initialSigmaYaw,
            predictSigmaX, predictSigmaY, predictSigmaYaw);

    dataRegistration_.setParameters(mapLengthX, mapLengthY, mapResolution,
            minElevation, maxElevation);
}

void GaSlam::poseCallback(const Pose& poseGuess, const Pose& bodyToGroundTF) {
    if (!poseInitialized_) poseInitialized_ = true;

    const auto transformedPoseGuess = poseGuess * bodyToGroundTF;
    poseEstimation_.predictPose(transformedPoseGuess);

    dataRegistration_.translateMap(poseEstimation_.getPose());
}

void GaSlam::cloudCallback(
        const Cloud::ConstPtr& cloud,
        const Pose& sensorToBodyTF) {
    if (!poseInitialized_) return;

    const auto sensorToMapTF = poseEstimation_.getPose() * sensorToBodyTF;
    const auto mapParameters = dataRegistration_.getMap().getParameters();
    Cloud::Ptr processedCloud(new Cloud);
    Cloud::Ptr mapCloud(new Cloud);
    std::vector<float> cloudVariances;

    CloudProcessing::processCloud(cloud, processedCloud, cloudVariances,
            sensorToMapTF, mapParameters, voxelSize_);

    std::unique_lock<std::mutex> mapGuard(dataRegistration_.getMapMutex());
    CloudProcessing::convertMapToCloud(dataRegistration_.getMap(), mapCloud);
    mapGuard.unlock();

    if (isFutureReady(filterPoseFuture_))
        filterPoseFuture_ = std::async(std::launch::async,
                &PoseEstimation::filterPose, &poseEstimation_,
                processedCloud, mapCloud);

    dataRegistration_.updateMap(processedCloud, cloudVariances);
}

}  // namespace ga_slam

