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
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
        double traversedDistanceThreshold, double slopeSumThreshold) {
    voxelSize_ = voxelSize;

    poseEstimation_.setParameters(numParticles, resampleFrequency,
            initialSigmaX, initialSigmaY, initialSigmaYaw,
            predictSigmaX, predictSigmaY, predictSigmaYaw);

    poseCorrection_.setParameters(traversedDistanceThreshold,
            slopeSumThreshold);

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
    std::vector<float> cloudVariances;

    CloudProcessing::processCloud(cloud, processedCloud, cloudVariances,
            sensorToMapTF, mapParameters, voxelSize_);

    if (isFutureReady(filterPoseFuture_))
        // Capture processedCloud (ptr) by value
        filterPoseFuture_ = std::async(std::launch::async, [&, processedCloud] {
            Cloud::Ptr mapCloud(new Cloud);

            std::unique_lock<std::mutex> guard(dataRegistration_.getMapMutex());
            const auto& map = dataRegistration_.getMap();
            CloudProcessing::convertMapToCloud(map, mapCloud);
            guard.unlock();

            poseEstimation_.filterPose(processedCloud, mapCloud);
        });

    dataRegistration_.updateMap(processedCloud, cloudVariances);

    if (isFutureReady(poseCorrectionFuture_))
        filterPoseFuture_ = std::async(std::launch::async, [&] {
            const auto pose = poseEstimation_.getPose();
            if (!poseCorrection_.distanceCriterionFulfilled(pose)) return;

            std::unique_lock<std::mutex> guard(dataRegistration_.getMapMutex());
            const auto& map = dataRegistration_.getMap();
            if (!poseCorrection_.featureCriterionFulfilled(map)) return;

            const auto correctedPose = poseCorrection_.matchMaps(pose, map);
            guard.unlock();

            poseEstimation_.predictPose(correctedPose);
        });
}

void GaSlam::registerOrbiterCloud(const Cloud::ConstPtr& cloud) {
    poseCorrection_.createGlobalMap(cloud);
}

}  // namespace ga_slam

