#include "ga_slam/GaSlam.hpp"

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/processing/CloudProcessing.hpp"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>
#include <future>

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_(),
          poseInitialized_(false) {
}

void GaSlam::configure(
        double mapLengthX, double mapLengthY, double mapResolution,
        double minElevation, double maxElevation, double voxelSize,
        int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
        double traversedDistanceThreshold, double minSlopeThreshold,
        double slopeSumThresholdMultiplier) {
    voxelSize_ = voxelSize;

    poseEstimation_.configure(numParticles, resampleFrequency,
            initialSigmaX, initialSigmaY, initialSigmaYaw,
            predictSigmaX, predictSigmaY, predictSigmaYaw);

    poseCorrection_.configure(traversedDistanceThreshold, minSlopeThreshold,
            slopeSumThresholdMultiplier);

    dataRegistration_.configure(mapLengthX, mapLengthY, mapResolution,
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

    dataRegistration_.updateMap(processedCloud, cloudVariances);

    if (isFutureReady(scanToMapMatchingFuture_))
        scanToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToRawCloud, this, processedCloud);

    if (isFutureReady(mapToMapMatchingFuture_))
        mapToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToGlobalMap, this);
}

void GaSlam::matchLocalMapToRawCloud(const Cloud::ConstPtr& rawCloud) {
    Cloud::Ptr mapCloud(new Cloud);

    std::unique_lock<std::mutex> guard(dataRegistration_.getMapMutex());
    const auto& map = dataRegistration_.getMap();
    CloudProcessing::convertMapToCloud(map, mapCloud);
    guard.unlock();

    poseEstimation_.filterPose(rawCloud, mapCloud);
}

void GaSlam::matchLocalMapToGlobalMap(void) {
    const auto pose = poseEstimation_.getPose();
    if (!poseCorrection_.distanceCriterionFulfilled(pose)) return;

    std::unique_lock<std::mutex> guard(dataRegistration_.getMapMutex());
    const auto& map = dataRegistration_.getMap();
    if (!poseCorrection_.featureCriterionFulfilled(map)) return;

    const auto correctedPose = poseCorrection_.matchMaps(pose, map);
    guard.unlock();

    poseEstimation_.predictPose(correctedPose);
}

void GaSlam::registerOrbiterCloud(const Cloud::ConstPtr& cloud) {
    poseCorrection_.createGlobalMap(cloud);
}

}  // namespace ga_slam

