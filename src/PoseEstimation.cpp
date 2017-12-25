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

    Eigen::Vector3d estimateTranslation = poseGuess.translation();
    Eigen::Vector3d estimateAngles = getAnglesFromPose(poseGuess);
    const auto deltaAngles = estimateAngles - getAnglesFromPose(pose_);
    const auto deltaTranslation = estimateTranslation - pose_.translation();

    particleFilter_.predict(deltaTranslation(0), deltaTranslation(1),
            deltaAngles(2));
    particleFilter_.update(cloud, mapCloud);
    particleFilter_.resample();
    particleFilter_.getEstimate(estimateTranslation(0), estimateTranslation(1),
            estimateAngles(2));

    pose_ = createPose(estimateTranslation, estimateAngles);
}

Pose PoseEstimation::createPose(
        const Eigen::Vector3d& translation,
        const Eigen::Vector3d& angles) {
    const auto rotation = Eigen::Quaterniond(
            Eigen::AngleAxisd(angles(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(angles(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitX()));

    Pose pose = Pose::Identity();
    pose.translation() = translation;
    pose.linear() = rotation.toRotationMatrix();

    return pose;
}

Eigen::Vector3d PoseEstimation::getAnglesFromPose(const Pose& pose) {
    double roll = pose.linear().eulerAngles(2, 1, 0)[2];
    double pitch = pose.linear().eulerAngles(2, 1, 0)[1];
    double yaw = pose.linear().eulerAngles(2, 1, 0)[0];

    return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace ga_slam

