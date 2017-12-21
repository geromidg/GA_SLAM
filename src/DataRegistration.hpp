#pragma once

#include "ga_slam/TypeDefs.hpp"

namespace ga_slam {

class DataRegistration {
  public:
    DataRegistration(void);

    DataRegistration(const DataRegistration&) = delete;
    DataRegistration& operator=(const DataRegistration&) = delete;
    DataRegistration(DataRegistration&&) = delete;
    DataRegistration& operator=(DataRegistration&&) = delete;

    const Map& getMap(void) const { return map_; }

    const Cloud::ConstPtr& getProcessedCloud(void) const {
            return processedCloud_; }

    bool setParameters(
            double mapSizeX, double mapSizeY,
            double robotPositionX, double robotPositionY,
            double mapResolution, double voxelSize,
            double minElevation, double maxElevation);

    void registerData(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToMapTF,
            const Pose& estimatedPose);

  protected:
    void updateMap(void);

    static void fuseGaussians(
            float& mean1, float& variance1,
            const float& mean2, const float& variance2);

  protected:
    Map map_;

    Cloud::Ptr processedCloud_;
    std::vector<float> cloudVariances_;

    double voxelSize_;
};

}  // namespace ga_slam

