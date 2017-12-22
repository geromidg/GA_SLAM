#pragma once

#include "ga_slam/TypeDefs.hpp"

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

