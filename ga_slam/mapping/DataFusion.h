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

#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

namespace ga_slam {

/** Module responsible for fusing the local map's data using different methods
  * depending on the stage of the SLAM pipeline.
  */
class DataFusion {
  public:
    /// Instantiates the fused map structure
    DataFusion(void) : fusedMap_() {}

    /// Delete the default copy/move constructors and operators
    DataFusion(const DataFusion&) = delete;
    DataFusion& operator=(const DataFusion&) = delete;
    DataFusion(DataFusion&&) = delete;
    DataFusion& operator=(DataFusion&&) = delete;

    /// Returns the fused elevation map
    const Map& getFusedMap(void) const { return fusedMap_; }

  protected:
    /// Fused elevation map with mean and variance values
    Map fusedMap_;
};

}  // namespace ga_slam

