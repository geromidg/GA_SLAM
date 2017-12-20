#pragma once

#include "ga_slam/TypeDefs.hpp"

namespace ga_slam {

class PoseCorrection {
  public:
    PoseCorrection(void) {}

    PoseCorrection(const PoseCorrection&) = delete;
    PoseCorrection& operator=(const PoseCorrection&) = delete;
    PoseCorrection(PoseCorrection&&) = delete;
    PoseCorrection& operator=(PoseCorrection&&) = delete;

    const Map& getGlobalMap(void) const { return globalMap_; }

  protected:
    Map globalMap_;
};

}  // namespace ga_slam

