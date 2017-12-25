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
    double estimateX, estimateY, estimateYaw;
    Cloud::Ptr mapCloud(new Cloud);

    double deltaX = poseGuess.translation().x() - pose_.translation().x();
    double deltaY = poseGuess.translation().y() - pose_.translation().y();
    double deltaYaw = poseGuess.linear().eulerAngles(2, 1, 0)[0] -
            pose_.linear().eulerAngles(2, 1, 0)[0];

    double guessZ = poseGuess.translation().z();
    double guessRoll = poseGuess.linear().eulerAngles(2, 1, 0)[2];
    double guessPitch = poseGuess.linear().eulerAngles(2, 1, 0)[1];

    CloudProcessing::convertMapToCloud(map, mapCloud);

    particleFilter_.predict(deltaX, deltaY, deltaYaw);
    particleFilter_.update(cloud, mapCloud);
    particleFilter_.resample();
    particleFilter_.getEstimate(estimateX, estimateY, estimateYaw);

    pose_.translation() = Eigen::Vector3d(estimateX, estimateY, guessZ);
    pose_.linear() = Eigen::Quaterniond(
            Eigen::AngleAxisd(estimateYaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(guessPitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(guessRoll, Eigen::Vector3d::UnitX())
            ).toRotationMatrix();
}

Pose PoseEstimation::calculateDeltaPose(const Pose& pose1, const Pose& pose2) {
    Pose deltaPose;

    deltaPose.translation() = pose1.translation() - pose2.translation();
    deltaPose.linear() = pose1.linear() - pose2.linear();

    return deltaPose;
}

}  // namespace ga_slam

