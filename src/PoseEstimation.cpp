#include "ga_slam/PoseEstimation.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

void PoseEstimation::setParameters(int numParticles) {
    particleFilter_.setParameters(numParticles);
    particleFilter_.initialize();
}

void PoseEstimation::estimatePose(
            const Map& map,
            const Cloud::ConstPtr& cloud,
            const Pose& poseGuess) {
    Cloud::Ptr mapCloud(new Cloud);
    CloudProcessing::convertMapToCloud(map, mapCloud);

    particleFilter_.predict(calculateDeltaPose(poseGuess, pose_));
    particleFilter_.update(cloud, mapCloud);
    particleFilter_.resample();
    pose_ = particleFilter_.getEstimate();
}

Pose PoseEstimation::calculateDeltaPose(const Pose& pose1, const Pose& pose2) {
    Pose deltaPose;

    deltaPose.translation() = pose1.translation() - pose2.translation();
    deltaPose.linear() = pose1.linear() - pose2.linear();

    return deltaPose;
}

}  // namespace ga_slam

