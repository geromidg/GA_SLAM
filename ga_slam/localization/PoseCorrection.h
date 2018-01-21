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

/** Module responsible for storing a pre-calculated row-resolution elevation
  * global map and using it to correct the increasing drift of the localization.
  */
class PoseCorrection {
  public:
    /// Instantiates the global elevation map
    PoseCorrection(void)
        : globalMapInitialized_(false),
          globalMap_(),
          globalMapPose_(Pose::Identity()),
          lastCorrectedPose_(Pose::Identity()) {}

    /// Delete the default copy/move constructors and operators
    PoseCorrection(const PoseCorrection&) = delete;
    PoseCorrection& operator=(const PoseCorrection&) = delete;
    PoseCorrection(PoseCorrection&&) = delete;
    PoseCorrection& operator=(PoseCorrection&&) = delete;

    /// Returns the global elevation map
    const Map& getGlobalMap(void) const { return globalMap_; }

    /// Returns the mutex protecting the map
    std::mutex& getGlobalMapMutex(void) { return globalMapMutex_; }

    /** Configures the global map and sets the module's parameters
      * @param[in] traversedDistanceThreshold the distance the robot must
      *            traverse before a new pose correction
      * @param[in] minSlopeThreshold the minimum slope (in radians) a cell of
      *            the map needs to have to be considered as elevation feature
      * @param[in] slopeSumThresholdMultiplier the value divided by the
      *            resolution of the map to produce the slope sum threshold,
      *            which determines if the map has enough elevation features
      * @param[in] matchAcceptanceThreshold the minimum score the matched
      *            position from template matching must have, in order for the
      *            matching to be accepted
      * @param[in] globalMapLength the size of one dimension of the global map
      * @param[in] globalMapResolution the resolution of the global map
      *            in meters
      */
    void configure(
            double traversedDistanceThreshold,
            double minSlopeThreshold,
            double slopeSumThresholdMultiplier,
            double matchAcceptanceThreshold,
            double globalMapLength,
            double globalMapResolution);

    /** Creates the global map by registering the global point cloud and
      * translating it to its corresponding pose
      * @param[in] globalCloud the global point cloud to be registered
      * @param[in] globalCloudPose the pose corresponding to the point cloud
      */
    void createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose);

    /** Checks if the robot has traversed enough distance before a new pose
      * correction can be applied
      * @param[in] pose the current robot's pose
      * @return true if the criterion is fulfilled
      */
    bool distanceCriterionFulfilled(const Pose& pose) const;

    /** Checks if the robot's current local elevation map has enough elevation
      * features before a new pose correction can be applied
      * @param[in] localMap the current robot's map
      * @return true if the criterion is fulfilled
      */
    bool featureCriterionFulfilled(const Map& localMap) const;

    /** Matches the local and global maps and corrects the pose if a match
      * is found
      * @param[in] localMap the current robot's map to be matched
      * @param[in] currentPose the current robot's pose
      * @param[out] correctedPose the corrected robot's pose
      * @return true if a match was found
      */
    bool matchMaps(
            const Map& localMap,
            const Pose& currentPose,
            Pose& correctedPose);

  protected:
    /// Whether the global map has been initialized
    std::atomic<bool> globalMapInitialized_;

    /// Global low-resolution elevation map
    Map globalMap_;

    /// Mutex protecting the map
    mutable std::mutex globalMapMutex_;

    /// Pose corresponding to the center of the global map
    Pose globalMapPose_;

    /// Last pose when a correction happened
    Pose lastCorrectedPose_;

    /// Distance the robot must traversed before a new correction is considered
    double traversedDistanceThreshold_;

    /// Minimum slope a cell of the map needs to have to be considered a feature
    double minSlopeThreshold_;

    /// Multiplier to adjust the slope threshold a map must reach before a
    /// new correction is considered
    double slopeSumThresholdMultiplier_;

    /// Minimum score in template matching a match must have to be accepted
    double matchAcceptanceThreshold_;
};

}  // namespace ga_slam

