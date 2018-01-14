#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <mutex>

namespace ga_slam {

class PoseCorrection {
  public:
    PoseCorrection(void)
        : globalMap_(),
          lastCorrectedPose_(Pose::Identity()) {}

    PoseCorrection(const PoseCorrection&) = delete;
    PoseCorrection& operator=(const PoseCorrection&) = delete;
    PoseCorrection(PoseCorrection&&) = delete;
    PoseCorrection& operator=(PoseCorrection&&) = delete;

    const Map& getGlobalMap(void) const { return globalMap_; }

    std::mutex& getGlobalMapMutex(void) { return globalMapMutex_; }

    void setParameters(
            double traversedDistanceThreshold,
            double minSlopeThreshold,
            double slopeSumThresholdMultiplier);

    void createGlobalMap(const Cloud::ConstPtr& cloud);

    bool distanceCriterionFulfilled(const Pose& pose) const;

    bool featureCriterionFulfilled(const Map& localMap) const;

    Pose matchMaps(const Pose& pose, const Map& localMap);

  protected:
    Map globalMap_;
    mutable std::mutex globalMapMutex_;

    Pose lastCorrectedPose_;

    double traversedDistanceThreshold_;
    double minSlopeThreshold_;
    double slopeSumThresholdMultiplier_;
};

}  // namespace ga_slam

