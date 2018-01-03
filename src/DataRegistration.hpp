#pragma once

#include "ga_slam/TypeDefs.hpp"

namespace ga_slam {

class DataRegistration {
  public:
    DataRegistration(void) : map_() {}

    DataRegistration(const DataRegistration&) = delete;
    DataRegistration& operator=(const DataRegistration&) = delete;
    DataRegistration(DataRegistration&&) = delete;
    DataRegistration& operator=(DataRegistration&&) = delete;

    const Map& getMap(void) const { return map_; }

    void setParameters(
            double mapLengthX, double mapLengthY, double mapResolution,
            double minElevation, double maxElevation);

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
};

}  // namespace ga_slam

