#include "ga_slam/localization/PoseEstimation.hpp"

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/processing/CloudProcessing.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>

namespace ga_slam {

void PoseEstimation::setParameters(
        int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    particleFilter_.setParameters(numParticles, initialSigmaX, initialSigmaY,
            initialSigmaYaw, predictSigmaX, predictSigmaY, predictSigmaYaw);
    particleFilter_.initialize();

    resampleFrequency_ = resampleFrequency;
}

void PoseEstimation::predictPose(const Pose& poseGuess) {
    Eigen::Vector3d estimateTranslation = poseGuess.translation();
    Eigen::Vector3d estimateAngles = getAnglesFromPose(poseGuess);

    std::unique_lock<std::mutex> guard(poseMutex_);
    const auto deltaAngles = estimateAngles - getAnglesFromPose(pose_);
    const auto deltaTranslation = estimateTranslation - pose_.translation();
    guard.unlock();

    particleFilter_.predict(deltaTranslation(0), deltaTranslation(1),
            deltaAngles(2));

    particleFilter_.getEstimate(estimateTranslation(0), estimateTranslation(1),
            estimateAngles(2));

    const Pose newPose = createPose(estimateTranslation, estimateAngles);

    guard.lock();
    pose_ = newPose;
    guard.unlock();
}

void PoseEstimation::filterPose(
        const Cloud::ConstPtr& rawCloud,
        const Cloud::ConstPtr& mapCloud) {
    std::unique_lock<std::mutex> guard(poseMutex_);
    const Pose lastPose = pose_;
    guard.unlock();

    particleFilter_.update(lastPose, rawCloud, mapCloud);

    resampleCounter_++;
    if (resampleCounter_ != resampleFrequency_) return;

    particleFilter_.resample();
    resampleCounter_ = 0;
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

