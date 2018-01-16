// GA SLAM
#include "ga_slam/localization/ParticleFilter.hpp"

// GTest
#include "gtest/gtest.h"

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

