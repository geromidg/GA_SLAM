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

#include "ga_slam/mapping/DataRegistration.h"

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
#include <cmath>
#include <mutex>

namespace ga_slam {

void DataRegistration::configure(
        double mapLength,
        double mapResolution,
        double minElevation,
        double maxElevation) {
    std::lock_guard<std::mutex> guard(mapMutex_);
    map_.setParameters(mapLength, mapResolution, minElevation, maxElevation);
}

void DataRegistration::translateMap(const Pose& estimatedPose, bool moveData) {
    std::lock_guard<std::mutex> guard(mapMutex_);
    map_.translate(estimatedPose.translation(), moveData);
}

void DataRegistration::updateMap(
        const Cloud::ConstPtr& cloud,
        const std::vector<float>& cloudVariances) {
    std::lock_guard<std::mutex> guard(mapMutex_);

    auto& meanData = map_.getMeanZ();
    auto& varianceData = map_.getVarianceZ();

    size_t cloudIndex = 0;
    size_t mapIndex;

    for (const auto& point : cloud->points) {
        cloudIndex++;

        if (!map_.getIndexFromPosition(point.x, point.y, mapIndex)) continue;

        float& mean = meanData(mapIndex);
        float& variance = varianceData(mapIndex);
        const float& pointVariance = cloudVariances[cloudIndex - 1];

        if (!std::isfinite(mean)) {
            mean = point.z;
            variance = pointVariance;
        } else {
            fuseGaussians(mean, variance, point.z, pointVariance);
        }
    }

    map_.setValid(true);
    map_.setTimestamp(cloud->header.stamp);
}

void DataRegistration::fuseGaussians(
        float& mean1, float& variance1,
        const float& mean2, const float& variance2) {
    const double innovation = mean2 - mean1;
    const double gain = variance1 / (variance1 + variance2);

    mean1 = mean1 + (gain * innovation);
    variance1 = variance1 * (1. - gain);
}

}  // namespace ga_slam

