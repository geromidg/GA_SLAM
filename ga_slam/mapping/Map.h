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

// Grid Map
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

// Eigen
#include <Eigen/Core>

// STL
#include <limits>

namespace ga_slam {

using GridMap = grid_map::GridMap;
using Matrix = Eigen::MatrixXf;
using Time = uint64_t;

struct MapParameters {
    double length;
    double size;
    double positionX;
    double positionY;
    double minElevation;
    double maxElevation;
    double resolution;
};

class Map {
  public:
    Map(void);

    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    Map(Map&&) = delete;
    Map& operator=(Map&&) = delete;

    grid_map::GridMapIterator begin(void) const {
        return grid_map::GridMapIterator(gridMap_); }

    const GridMap& getGridMap(void) const { return gridMap_; }

    void setValid(bool valid) { valid_ = valid; }

    bool isValid(void) const { return valid_; }

    const Matrix& getMeanZ(void) const { return gridMap_.get(layerMeanZ_); }

    Matrix& getMeanZ(void) { return gridMap_.get(layerMeanZ_); }

    const Matrix& getVarianceZ(void) const {
        return gridMap_.get(layerVarianceZ_); }

    Matrix& getVarianceZ(void) {
        return gridMap_.get(layerVarianceZ_); }

    void clear(void) { gridMap_.clearBasic(); }

    Time getTimestamp(void) const { return gridMap_.getTimestamp(); }

    void setTimestamp(const Time& time) { gridMap_.setTimestamp(time); }

    void setParameters(
            double length,
            double resolution,
            double minElevation = -std::numeric_limits<double>::max(),
            double maxElevation = std::numeric_limits<double>::max());

    MapParameters getParameters(void) const;

    bool getIndexFromPosition(
            double positionX,
            double positionY,
            size_t& index) const;

    void getPointFromArrayIndex(
            const grid_map::Index& arrayIndex,
            const Matrix& layerData,
            Eigen::Vector3d& point) const;

    void translate(const Eigen::Vector3d& translation, bool moveData = false);

  protected:
    GridMap gridMap_;

    bool valid_;

    double minElevation_;
    double maxElevation_;

    static constexpr const char* layerMeanZ_ = "meanZ";
    static constexpr const char* layerVarianceZ_ = "varZ";
};

}  // namespace ga_slam

