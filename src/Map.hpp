#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

namespace ga_slam {

using GridMap = grid_map::GridMap;
using Matrix = Eigen::MatrixXf;
using Time = uint64_t;

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

    double getMinElevation(void) const { return minElevation_; }

    double getMaxElevation(void) const { return maxElevation_; }

    double getPositionX(void) const { return gridMap_.getPosition().x(); }

    double getPositionY(void) const { return gridMap_.getPosition().y(); }

    double getLengthX(void) const { return gridMap_.getLength().x(); }

    double getLengthY(void) const { return gridMap_.getLength().y(); }

    int getSizeX(void) const { return gridMap_.getSize().x(); }

    int getSizeY(void) const { return gridMap_.getSize().y(); }

    const Matrix& getMeanZ(void) const { return gridMap_.get(layerMeanZ_); }

    Matrix& getMeanZ(void) { return gridMap_.get(layerMeanZ_); }

    const Matrix& getVarianceZ(void) const {
        return gridMap_.get(layerVarianceZ_); }

    Matrix& getVarianceZ(void) {
        return gridMap_.get(layerVarianceZ_); }

    Time getTimestamp(void) const { return gridMap_.getTimestamp(); }

    void setTimestamp(const Time& time) { gridMap_.setTimestamp(time); }

    void setMapParameters(
            double sizeX, double sizeY, double resolution,
            double minElevation, double maxElevation);

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

    double minElevation_;
    double maxElevation_;

    const std::string layerMeanZ_;
    const std::string layerVarianceZ_;
};

}  // namespace ga_slam

