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

class CloudProcessing {
  public:
    CloudProcessing(void) = delete;

    static void processCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            std::vector<float>& cloudVariances,
            const Pose& sensorToMapTF,
            const MapParameters& mapParameters,
            double voxelSize);

    static void downsampleCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            double voxelSize);

    static void transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf);

    static void cropCloudToMap(
            Cloud::Ptr& cloud,
            const MapParameters& mapParameters);

    static void calculateCloudVariances(
            const Cloud::ConstPtr& cloud,
            std::vector<float>& variances);

    static void convertMapToCloud(const Map& map, Cloud::Ptr& cloud);

    static double matchClouds(
            const Cloud::ConstPtr& cloud1,
            const Cloud::ConstPtr& cloud2);
};

}  // namespace ga_slam

