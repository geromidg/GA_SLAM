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

    void createGlobalMap(const Cloud::ConstPtr& cloud);

    bool distanceCriterionFulfilled(const Pose& pose) const;

    bool featureCriterionFulfilled(const Map& map) const;

    Pose matchMaps(const Pose& pose, const Map& map);

  protected:
    Map globalMap_;
    mutable std::mutex globalMapMutex_;

    Pose lastCorrectedPose_;
};

}  // namespace ga_slam

