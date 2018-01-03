#pragma once

#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/ParticleFilter.hpp"

namespace ga_slam {

class PoseEstimation {
  public:
    PoseEstimation(void)
            : pose_(Pose::Identity()),
              particleFilter_() {}

    PoseEstimation(const PoseEstimation&) = delete;
    PoseEstimation& operator=(const PoseEstimation&) = delete;
    PoseEstimation(PoseEstimation&&) = delete;
    PoseEstimation& operator=(PoseEstimation&&) = delete;

    const Pose& getPose(void) const { return pose_; }

    void setParameters(
            int numParticles,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    void predictPose(const Pose& poseGuess = Pose::Identity());

    void filterPose(const Map& map, const Cloud::ConstPtr& cloud);

  protected:
    static Pose createPose(
            const Eigen::Vector3d& translation,
            const Eigen::Vector3d& angles);

    static Eigen::Vector3d getAnglesFromPose(const Pose& pose);

  protected:
    Pose pose_;

    ParticleFilter particleFilter_;
};

}  // namespace ga_slam

