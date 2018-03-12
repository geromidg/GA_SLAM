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

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// STL
#include <string>
#include <cmath>

// GMock
#include "gmock/gmock.h"

namespace ga_slam {

class PoseCorrectionTest : public ::testing::Test {
  protected:
    PoseCorrectionTest(void) : gaSlam_() {
        const double traversedDistanceThreshold = 10.;
        const double minSlopeThreshold = 0.5;
        const double slopeSumThresholdMultiplier = 10.;
        const double matchAcceptanceThreshold = 0.9;
        const double matchYaw = true;
        const double matchYawRange = 20 * M_PI / 180;
        const double matchYawStep = M_PI / 180;
        const double globalMapLength = 100.;
        const double globalMapResolution = 1.;

        gaSlam_.configure(1., 1., 1., 1., 1., 0., 0., 1., 1, 1, 0., 0., 0., 0.,
                0., 0., traversedDistanceThreshold, minSlopeThreshold,
                slopeSumThresholdMultiplier, matchAcceptanceThreshold,
                matchYaw, matchYawRange, matchYawStep,
                globalMapLength, globalMapResolution);

        cloud_.reset(new Cloud);
    }

    void insertGlobalCloud(void) {
        pcl::io::loadPCDFile<pcl::PointXYZ>(filename_, *cloud_);
        gaSlam_.createGlobalMap(cloud_, Pose::Identity());
    }

    const Map& globalMap(void) const { return gaSlam_.getGlobalMap(); }

  private:
    GaSlam gaSlam_;

    Cloud::Ptr cloud_;

    /// Dataset path
    static constexpr const char* filename_ =
            "/tmp/ga_slam_test_data/cloud_sequence/global_cloud_0.pcd";
};

TEST_F(PoseCorrectionTest, RegisterGlobalCloud) {
    ASSERT_FALSE(globalMap().isValid());
    insertGlobalCloud();
    ASSERT_TRUE(globalMap().isValid());
}

} // namespace ga_slam

