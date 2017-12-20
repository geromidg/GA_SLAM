#include "ga_slam/PoseEstimation.hpp"

namespace ga_slam {

void PoseEstimation::estimatePose(const Pose& poseGuess) {
    pose_ = poseGuess;
}

}  // namespace ga_slam

