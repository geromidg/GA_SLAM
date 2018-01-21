/*
 * This file is part of GA SLAM.
 * Copyright (C) 2018 Dimitris Geromichalos,
 * Planetary Robotics Lab (PRL), European Space Agency (ESA)
 *
 * GA SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GA SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GA SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <mutex>
#include <atomic>

namespace ga_slam {

class PoseCorrection {
  public:
    PoseCorrection(void)
        : globalMapInitialized_(false),
          globalMap_(),
          globalMapPose_(Pose::Identity()),
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
            double matchAcceptanceThreshold,
            double globalMapLength,
            double globalMapResolution);

    void createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose);

    bool distanceCriterionFulfilled(const Pose& pose) const;

    bool featureCriterionFulfilled(const Map& localMap) const;

    bool matchMaps(
            const Map& localMap,
            const Pose& currentPose,
            Pose& correctedPose);

  protected:
    std::atomic<bool> globalMapInitialized_;

    Map globalMap_;
    mutable std::mutex globalMapMutex_;

    Pose globalMapPose_;
    Pose lastCorrectedPose_;

    double traversedDistanceThreshold_;
    double minSlopeThreshold_;
    double slopeSumThresholdMultiplier_;
    double matchAcceptanceThreshold_;
};

}  // namespace ga_slam

