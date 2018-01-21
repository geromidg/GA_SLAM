// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/GaSlam.h"

// GTest
#include "gtest/gtest.h"

namespace ga_slam {

TEST(GaSlamTest, MapParameters) {
    GaSlam gaSlam;

    const double mapResolution = 3.14;
    gaSlam.configure(1., mapResolution, 1., 1., 1., 1, 1, 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1., 1., 1.);

    const auto& map = gaSlam.getRawMap();
    const auto mapParameters = map.getParameters();

    ASSERT_EQ(mapParameters.resolution, mapResolution);
}

} // namespace ga_slam

