#pragma once

#include "grid_map_core/GridMap.hpp"

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

    const GridMap& getGridMap(void) const { return gridMap_; }

    double getMinElevation(void) const { return minElevation_; }

    double getMaxElevation(void) const { return maxElevation_; }

    const Matrix& getMeanZ(void) const { return gridMap_.get(layerMeanZ_); }

    Matrix& getMeanZ(void) { return gridMap_.get(layerMeanZ_); }

    const Matrix& getVarianceZ(void) const {
            return gridMap_.get(layerVarianceZ_); }

    Matrix& getVarianceZ(void) {
            return gridMap_.get(layerVarianceZ_); }

    Time getTimestamp(void) const { return timestamp_; }

    void setTimestamp(const Time& timestamp) { timestamp_ = timestamp; }

    bool setParameters(
            double mapSizeX, double mapSizeY,
            double robotPositionX, double robotPositionY,
            double mapResolution, double minElevation, double maxElevation);

    void translate(const Eigen::Vector3d& translation);

  protected:
    GridMap gridMap_;

    Time timestamp_;

    double mapSizeX_;
    double mapSizeY_;
    double robotPositionX_;
    double robotPositionY_;
    double mapResolution_;
    double minElevation_;
    double maxElevation_;

    const std::string layerMeanZ_;
    const std::string layerVarianceZ_;
};

}  // namespace ga_slam

