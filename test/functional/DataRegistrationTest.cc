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

// GMock
#include "gmock/gmock.h"

namespace ga_slam {

class DataRegistrationTest : public ::testing::Test {
  protected:
    DataRegistrationTest(void) : gaSlam_(),
              sequenceCount_(0) {
        const double mapLength = 20.;
        const double mapResolution = 0.2;
        const double minElevation = -2.;
        const double maxElevation = 2.;
        const double voxelSize = mapResolution;

        gaSlam_.configure(mapLength, mapResolution, minElevation, maxElevation,
                voxelSize, 1, 1, 0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 1.);

        cloud_.reset(new Cloud);
    }

    void insertZeroPose(void) {
        gaSlam_.poseCallback(Pose::Identity());
    }

    void insertSingleCloud(void) {
        readNextCloud();
        gaSlam_.cloudCallback(cloud_);
    }

    const Map& map(void) const { return gaSlam_.getRawMap(); }

  private:
    void readNextCloud(void) {
        const std::string filename = filenamePrefix_ +
                std::to_string(sequenceCount_) + filenameSuffix_;
        pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud_);
        sequenceCount_++;
    }

  private:
    GaSlam gaSlam_;

    Cloud::Ptr cloud_;

    /// Dataset path and size
    static constexpr const char* filenamePrefix_ =
            "/tmp/ga_slam_test_data/cloud_sequence/local_cloud_";
    static constexpr const char* filenameSuffix_ = ".pcd";

    /// Current file of dataset
    int sequenceCount_;
};

TEST_F(DataRegistrationTest, RegisterSingleCloud) {
    ASSERT_FALSE(map().isValid());
    insertZeroPose();
    ASSERT_FALSE(map().isValid());
    insertSingleCloud();
    ASSERT_TRUE(map().isValid());
}

}

} // namespace ga_slam

