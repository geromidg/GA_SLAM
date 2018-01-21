#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

namespace ga_slam {

class DataFusion {
  public:
    DataFusion(void) {}

    DataFusion(const DataFusion&) = delete;
    DataFusion& operator=(const DataFusion&) = delete;
    DataFusion(DataFusion&&) = delete;
    DataFusion& operator=(DataFusion&&) = delete;

    const Map& getFusedMap(void) const { return fusedMap_; }

  protected:
    Map fusedMap_;
};

}  // namespace ga_slam

