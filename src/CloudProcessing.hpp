#pragma once

#include "ga_slam/TypeDefs.hpp"

namespace ga_slam {

class CloudProcessing {
  public:
    CloudProcessing(void) = delete;

    static void processCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            std::vector<float>& cloudVariances,
            const Pose& sensorToMapTF,
            const Map& map,
            double voxelSize);

    static void downsampleCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            double voxelSize);

    static void transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf);

    static void cropCloudToMap(
            Cloud::Ptr& cloud,
            const Map& map);

    static void calculateCloudVariances(
            const Cloud::ConstPtr& cloud,
            std::vector<float>& variances);

    static void convertMapToCloud(const Map& map, Cloud::Ptr& cloud);

    static double measureCloudAlignment(
            const Cloud::ConstPtr& cloud1,
            const Cloud::ConstPtr& cloud2);
};

}  // namespace ga_slam

