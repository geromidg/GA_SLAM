#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/mapping/Map.hpp"

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
            double mapLengthX, double mapLengthY, double mapResolution,
            double minElevation, double maxElevation);

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

