#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/mapping/Map.hpp"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
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

    void configure(
            double traversedDistanceThreshold,
            double minSlopeThreshold,
            double slopeSumThresholdMultiplier,
            double globalMapLength,
            double globalMapResolution);

    void createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalPose);

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

