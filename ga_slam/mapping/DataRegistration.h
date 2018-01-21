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

class DataRegistration {
  public:
    DataRegistration(void) : map_() {}

    DataRegistration(const DataRegistration&) = delete;
    DataRegistration& operator=(const DataRegistration&) = delete;
    DataRegistration(DataRegistration&&) = delete;
    DataRegistration& operator=(DataRegistration&&) = delete;

    const Map& getMap(void) const { return map_; }

    std::mutex& getMapMutex(void) { return mapMutex_; }

    void configure(
            double mapLength,
            double mapResolution,
            double minElevation,
            double maxElevation);

    MapParameters getMapParameters(void) const {
        std::lock_guard<std::mutex> guard(mapMutex_);
        return map_.getParameters();
    }

    void translateMap(const Pose& estimatedPose);

    void updateMap(
            const Cloud::ConstPtr& cloud,
            const std::vector<float>& cloudVariances);

  protected:
    static void fuseGaussians(
            float& mean1, float& variance1,
            const float& mean2, const float& variance2);

  protected:
    Map map_;
    mutable std::mutex mapMutex_;
};

}  // namespace ga_slam

