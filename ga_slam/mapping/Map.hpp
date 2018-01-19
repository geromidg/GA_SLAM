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

    void translate(const Eigen::Vector3d& translation);

  protected:
    GridMap gridMap_;

    bool valid_;

    double minElevation_;
    double maxElevation_;

    static constexpr const char* layerMeanZ_ = "meanZ";
    static constexpr const char* layerVarianceZ_ = "varZ";
};

}  // namespace ga_slam

