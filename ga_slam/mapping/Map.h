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

/// Contains the map's parameters which are needed by other modules
struct MapParameters {
    /// Size of one dimension of the map in meters
    double length;

    /// Size of one dimension of the map in number of cells
    double size;

    /// Current position of the map in the world
    double positionX;
    double positionY;

    /// Minimum and maximum elevation values the map can handle
    double minElevation;
    double maxElevation;

    /// Resolution of each square cell of the map in meters
    double resolution;
};

/** Wrapper for the GridMap class that extends its functionality in the
  * context of the GA SLAM library.
  */
class Map {
  public:
    /// Initializes the GridMap instance by setting the map's layers
    Map(void);

    /// Delete the default copy/move constructors and operators
    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    Map(Map&&) = delete;
    Map& operator=(Map&&) = delete;

    /// Returns an iterator of the GridMap instance
    grid_map::GridMapIterator begin(void) const {
        return grid_map::GridMapIterator(gridMap_); }

    /// Returns the GridMap instance
    const GridMap& getGridMap(void) const { return gridMap_; }

    /// Validates or invalidates the map by setting the valid flag
    void setValid(bool valid) { valid_ = valid; }

    /// Returns the valid flag
    bool isValid(void) const { return valid_; }

    /// Return the constant matrix containing the mean elevation data layer
    const Matrix& getMeanZ(void) const { return gridMap_.get(layerMeanZ_); }

    /// Return the matrix containing the mean elevation data layer
    Matrix& getMeanZ(void) { return gridMap_.get(layerMeanZ_); }

    /// Return the constant matrix containing the variance elevation data layer
    const Matrix& getVarianceZ(void) const {
        return gridMap_.get(layerVarianceZ_); }

    /// Return the matrix containing the variance elevation data layer
    Matrix& getVarianceZ(void) {
        return gridMap_.get(layerVarianceZ_); }

    /// Returns the timestamp of the map
    Time getTimestamp(void) const { return gridMap_.getTimestamp(); }

    /// Sets the timestamp of the map
    void setTimestamp(const Time& time) { gridMap_.setTimestamp(time); }

    /// Clears the values of all cells of the mean and variances layers
    void clear(void) { gridMap_.clearBasic(); }

    /** Configures the initial GridMap instance
      * @param[in] length the length of the GridMap
      * @param[in] resolution the resolution of the GridMap
      * @param[in] minElevation the minimum elevation value
      * @param[in] maxElevation the maximum elevation value
      */
    void setParameters(
            double length,
            double resolution,
            double minElevation,
            double maxElevation);

    /// Returns the structure containing the map's parameters
    MapParameters getParameters(void) const;

    /** Finds the map's linear index that corresponds to a specific position
      * @param[in] positionX the x coordinate of the position
      * @param[in] positionY the y coordinate of the position
      * @param[out] index the linear index matching to the position
      * @return true if the index was found
      */
    bool getIndexFromPosition(
            double positionX,
            double positionY,
            size_t& index) const;

    /** Finds the 3D point that corresponds to the map's array (2D) index
      * @param[in] arrayIndex the array index of the point
      * @param[in] layerData the layer's data matrix
      * @param[out] point the 3D point corresponding to the array index
      */
    void getPointFromArrayIndex(
            const grid_map::Index& arrayIndex,
            const Matrix& layerData,
            Eigen::Vector3d& point) const;

    /** Translates the map by either emptying the cells that fall out of the
      * map using a two-dimensional circular buffer or by simply updating the
      * position of the map in the world frame and keeping the data as it is
      * @param[in] translation the translation to be applied to the map
      * @param[in] moveData whether to move and keep the data or empty the
      *            cells that fall out of the map
      */
    void translate(const Eigen::Vector3d& translation, bool moveData);

  protected:
    /// Instance of the wrapped GridMap class
    GridMap gridMap_;

    /// Whether the map is valid
    bool valid_;

    /// Minimum and maximum elevation values the map can hold
    double minElevation_;
    double maxElevation_;

    /// Names of the map's data layers for the mean and variance elevation
    static constexpr const char* layerMeanZ_ = "meanZ";
    static constexpr const char* layerVarianceZ_ = "varZ";
};

}  // namespace ga_slam

