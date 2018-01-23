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

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/GaSlam.h"

// Eigen
#include <Eigen/Geometry>

// GTest
#include "gtest/gtest.h"

namespace ga_slam {

TEST(GaSlamTest, MapParameters) {
    GaSlam gaSlam;

    const double mapResolution = 3.14;
    gaSlam.configure(1., mapResolution, 1., 1., 1., 1, 1, 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1., 1., 1., Pose::Identity());

    const auto& map = gaSlam.getRawMap();
    const auto mapParameters = map.getParameters();

    ASSERT_EQ(mapParameters.resolution, mapResolution);
}

} // namespace ga_slam

