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

/** TODO
  */
struct MapParameters {
    /// TODO
    double length;

    /// TODO
    double size;

    /// TODO
    double positionX;
    double positionY;

    /// TODO
    double minElevation;
    double maxElevation;

    /// TODO
    double resolution;
};

/** TODO
  */
class Map {
  public:
    /// TODO
    Map(void);

    /// TODO
    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    Map(Map&&) = delete;
    Map& operator=(Map&&) = delete;

    /// TODO
    grid_map::GridMapIterator begin(void) const {
        return grid_map::GridMapIterator(gridMap_); }

    /// TODO
    const GridMap& getGridMap(void) const { return gridMap_; }

    /// TODO
    void setValid(bool valid) { valid_ = valid; }

    /// TODO
    bool isValid(void) const { return valid_; }

    /// TODO
    const Matrix& getMeanZ(void) const { return gridMap_.get(layerMeanZ_); }

    /// TODO
    Matrix& getMeanZ(void) { return gridMap_.get(layerMeanZ_); }

    /// TODO
    const Matrix& getVarianceZ(void) const {
        return gridMap_.get(layerVarianceZ_); }

    /// TODO
    Matrix& getVarianceZ(void) {
        return gridMap_.get(layerVarianceZ_); }

    /// TODO
    Time getTimestamp(void) const { return gridMap_.getTimestamp(); }

    /// TODO
    void setTimestamp(const Time& time) { gridMap_.setTimestamp(time); }

    /// TODO
    void clear(void) { gridMap_.clearBasic(); }

    /** TODO
      * @param[in] length TODO
      * @param[in] resolution TODO
      * @param[in] minElevation TODO
      * @param[in] maxElevation TODO
      */
    void setParameters(
            double length,
            double resolution,
            double minElevation = -std::numeric_limits<double>::max(),
            double maxElevation = std::numeric_limits<double>::max());

    /** TODO
      * @return TODO
      */
    MapParameters getParameters(void) const;

    /** TODO
      * @param[in] positionX TODO
      * @param[in] positionY TODO
      * @param[out] index TODO
      * @return TODO
      */
    bool getIndexFromPosition(
            double positionX,
            double positionY,
            size_t& index) const;

    /** TODO
      * @param[in] arrayIndex TODO
      * @param[in] layerData TODO
      * @param[out] point TODO
      */
    void getPointFromArrayIndex(
            const grid_map::Index& arrayIndex,
            const Matrix& layerData,
            Eigen::Vector3d& point) const;

    /** TODO
      * @param[in] translation TODO
      * @param[in] moveData TODO
      */
    void translate(const Eigen::Vector3d& translation, bool moveData = false);

  protected:
    /// TODO
    GridMap gridMap_;

    /// TODO
    bool valid_;

    /// TODO
    double minElevation_;
    double maxElevation_;

    /// TODO
    static constexpr const char* layerMeanZ_ = "meanZ";
    static constexpr const char* layerVarianceZ_ = "varZ";
};

}  // namespace ga_slam

