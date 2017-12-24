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

    void estimatePose(
            const Map& map,
            const Cloud::ConstPtr& cloud,
            const Pose& poseGuess = Pose::Identity());

  protected:
    static Pose calculateDeltaPose(const Pose& pose1, const Pose& pose2);

  protected:
    Pose pose_;

    ParticleFilter particleFilter_;
};

}  // namespace ga_slam

