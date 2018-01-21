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

namespace ga_slam {

/** TODO
  */
class CloudProcessing {
  public:
    /// TODO
    CloudProcessing(void) = delete;

    /** TODO
      * @param[in] inputCloud TODO
      * @param[out] outputCloud TODO
      * @param[out] cloudVariances TODO
      * @param[in] sensorToMapTF TODO
      * @param[in] mapParameters TODO
      * @param[in] voxelSize TODO
      */
    static void processCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            std::vector<float>& cloudVariances,
            const Pose& sensorToMapTF,
            const MapParameters& mapParameters,
            double voxelSize);

    /** TODO
      * @param[in] inputCloud TODO
      * @param[out] outputCloud TODO
      * @param[in] voxelSize TODO
      */
    static void downsampleCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            double voxelSize);

    /** TODO
      * @param[in/out] cloud TODO
      * @param[in] tf TODO
      */
    static void transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf);

    /** TODO
      * @param[in/out] cloud TODO
      * @param[in] mapParameters TODO
      */
    static void cropCloudToMap(
            Cloud::Ptr& cloud,
            const MapParameters& mapParameters);

    /** TODO
      * @param[in] cloud TODO
      * @param[out] variances TODO
      */
    static void calculateCloudVariances(
            const Cloud::ConstPtr& cloud,
            std::vector<float>& variances);

    /** TODO
      * @param[in] map TODO
      * @param[out] cloud TODO
      */
    static void convertMapToCloud(const Map& map, Cloud::Ptr& cloud);

    /** TODO
      * @param[in] cloud1 TODO
      * @param[in] cloud2 TODO
      * @return TODO
      */
    static double matchClouds(
            const Cloud::ConstPtr& cloud1,
            const Cloud::ConstPtr& cloud2);
};

}  // namespace ga_slam

