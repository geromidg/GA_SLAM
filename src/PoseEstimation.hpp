#pragma once

#include "ga_slam/TypeDefs.hpp"

namespace ga_slam {

class PoseEstimation {
  public:
    PoseEstimation(void) {}

    PoseEstimation(const PoseEstimation&) = delete;
    PoseEstimation& operator=(const PoseEstimation&) = delete;
    PoseEstimation(PoseEstimation&&) = delete;
    PoseEstimation& operator=(PoseEstimation&&) = delete;

    const Pose& getPose(void) const { return pose_; }

    void estimatePose(const Pose& poseGuess = Pose::Identity());

  protected:
    Pose pose_;
};

}  // namespace ga_slam

