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
#include <vector>
#include <mutex>

namespace ga_slam {

/** Module responsible for maintaining a local (robot-centric) elevation map
  * and registering input point clouds to it.
  */
class DataRegistration {
  public:
    /// Instantiates the local elevation map
    DataRegistration(void) : map_() {}

    /// Delete the default copy/move constructors and operators
    DataRegistration(const DataRegistration&) = delete;
    DataRegistration& operator=(const DataRegistration&) = delete;
    DataRegistration(DataRegistration&&) = delete;
    DataRegistration& operator=(DataRegistration&&) = delete;

    /// Returns the elevation map
    const Map& getMap(void) const { return map_; }

    /// Returns the mutex protecting the map
    std::mutex& getMapMutex(void) { return mapMutex_; }

    /** Configures the map by passing the parameters
      * @param[in] mapLength the size of one dimension of the map in meters
      * @param[in] mapResolution the resolution of the map in meters
      * @param[in] minElevation the minimum elevation value the map can hold
      * @param[in] maxElevation the maximum elevation value the map can hold
      */
    void configure(
            double mapLength,
            double mapResolution,
            double minElevation,
            double maxElevation);

    /// Returns the structure containing the map's parameters
    MapParameters getMapParameters(void) const {
        std::lock_guard<std::mutex> guard(mapMutex_);
        return map_.getParameters();
    }

    /** Translates the map to a new position
      * @param[in] estimatedPose the robot's new pose estimate
      */
    void translateMap(const Pose& estimatedPose);

    /** Registers a point cloud to the map by fusing each point of the cloud
      * to corresponding cell of the map. Each cell of the map and each point
      * of the input cloud are represented as gaussians (mean and variance),
      * so a gaussian fusion is performed.
      * @note the position of the cloud's points and the map's position should
      *       correspond to the same reference frame.
      * @param[in] cloud the point cloud to be registered
      * @param[in] cloudVariances the vector of the variances corresponding to
      *            each point of the cloud
      */
    void updateMap(
            const Cloud::ConstPtr& cloud,
            const std::vector<float>& cloudVariances);

  protected:
    /** Performs a fusion of two gaussians and stores the result in the first
      * @note the order of the gaussians does not matter
      * @param[in/out] mean1 the mean value of the first gaussian
      * @param[in/out] variance1 the variance value of the first gaussian
      * @param[in] mean2 the mean value of the second gaussian
      * @param[in] variance2 the variance value of the second gaussian
      */
    static void fuseGaussians(
            float& mean1, float& variance1,
            const float& mean2, const float& variance2);

  protected:
    /// Local (robot-centric) elevation map with mean and variance values
    Map map_;

    /// Mutex protecting the map
    mutable std::mutex mapMutex_;
};

}  // namespace ga_slam

