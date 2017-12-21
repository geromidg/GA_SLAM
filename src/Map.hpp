#pragma once

#include "grid_map_core/GridMap.hpp"

namespace ga_slam {

using GridMap = grid_map::GridMap;
using Matrix = Eigen::MatrixXf;
using Time = uint64_t;

class Map {
  public:
    Map(void)
            : layerMeanZ_("meanZ"),
              layerVarianceZ_("varZ") {
        gridMap_ = GridMap({layerMeanZ_, layerVarianceZ_});
        gridMap_.setBasicLayers({layerMeanZ_, layerVarianceZ_});
        gridMap_.clearBasic();
        gridMap_.resetTimestamp();
    }

    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    Map(Map&&) = delete;
    Map& operator=(Map&&) = delete;

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

    void setParameters(
            double sizeX, double sizeY,
            double positionX, double positionY,
            double resolution, double minElevation, double maxElevation) {
        minElevation_ = minElevation;
        maxElevation_ = maxElevation;

        gridMap_.setGeometry(grid_map::Length(sizeX, sizeY), resolution,
                grid_map::Position(positionX, positionY));
    }

    void translate(const Eigen::Vector3d& translation) {
        grid_map::Position positionXY(translation.x(), translation.y());

        gridMap_.move(positionXY);
    }

  protected:
    GridMap gridMap_;

    double minElevation_;
    double maxElevation_;

    const std::string layerMeanZ_;
    const std::string layerVarianceZ_;
};

}  // namespace ga_slam

