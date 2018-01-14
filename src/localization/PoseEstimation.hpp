#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/localization/ParticleFilter.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>
#include <atomic>

namespace ga_slam {

class PoseEstimation {
  public:
    PoseEstimation(void)
            : pose_(Pose::Identity()),
              resampleCounter_(0),
              particleFilter_() {}

    PoseEstimation(const PoseEstimation&) = delete;
    PoseEstimation& operator=(const PoseEstimation&) = delete;
    PoseEstimation(PoseEstimation&&) = delete;
    PoseEstimation& operator=(PoseEstimation&&) = delete;

    Pose getPose(void) const {
        std::lock_guard<std::mutex> guard(poseMutex_);
        return pose_;
    }

    std::mutex& getPoseMutex(void) { return poseMutex_; }

    void setParameters(
            int numParticles, int resampleFrequency,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    void predictPose(const Pose& poseGuess = Pose::Identity());

    void filterPose(
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

  protected:
    static Pose createPose(
            const Eigen::Vector3d& translation,
            const Eigen::Vector3d& angles);

    static Eigen::Vector3d getAnglesFromPose(const Pose& pose);

  protected:
    Pose pose_;
    mutable std::mutex poseMutex_;

    std::atomic<int> resampleCounter_;
    int resampleFrequency_;

    ParticleFilter particleFilter_;
};

}  // namespace ga_slam

