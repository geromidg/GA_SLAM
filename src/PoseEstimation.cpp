#include "ga_slam/PoseEstimation.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

void PoseEstimation::setParameters(
        int numParticles,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    particleFilter_.setParameters(numParticles, initialSigmaX, initialSigmaY,
            initialSigmaYaw, predictSigmaX, predictSigmaY, predictSigmaYaw);
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

    pose_.rotate(Eigen::AngleAxisd(poseGuess.linear().eulerAngles(2, 1, 0)[2],
                Eigen::Vector3d::UnitX()));
    pose_.rotate(Eigen::AngleAxisd(poseGuess.linear().eulerAngles(2, 1, 0)[1],
                Eigen::Vector3d::UnitY()));
}

Pose PoseEstimation::calculateDeltaPose(const Pose& pose1, const Pose& pose2) {
    Pose deltaPose;

    deltaPose.translation() = pose1.translation() - pose2.translation();
    deltaPose.linear() = pose1.linear() - pose2.linear();

    return deltaPose;
}

}  // namespace ga_slam

