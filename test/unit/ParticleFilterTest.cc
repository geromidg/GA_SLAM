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
#include "ga_slam/localization/ParticleFilter.h"

// GMock
#include "gmock/gmock.h"

namespace ga_slam {

TEST(ParticleFilterTest, Initialization) {
    ParticleFilter particleFilter;
    particleFilter.configure(1, 0., 0., 0., 0., 0., 0.);

    double initialX = 1.52, initialY = 2.72, initialYaw = 3.14;
    particleFilter.initialize(initialX, initialY, initialYaw);

    double estimateX = 1.52, estimateY = 2.72, estimateYaw = 3.14;
    particleFilter.getEstimate(estimateX, estimateY, estimateYaw);

    ASSERT_EQ(estimateX, initialX);
    ASSERT_EQ(estimateY, initialY);
    ASSERT_EQ(estimateYaw, initialYaw);
}

} // namespace ga_slam

