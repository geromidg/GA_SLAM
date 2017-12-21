#include "ga_slam/Map.hpp"

namespace ga_slam {

Map::Map(void)
        : timestamp_(0),
          layerMeanZ_("meanZ"),
          layerVarianceZ_("varZ") {
    gridMap_ = GridMap({layerMeanZ_, layerVarianceZ_});
    gridMap_.setBasicLayers({layerMeanZ_, layerVarianceZ_});
    gridMap_.clearBasic();
    gridMap_.resetTimestamp();
}

bool Map::setParameters(
        double mapSizeX, double mapSizeY,
        double robotPositionX, double robotPositionY,
        double mapResolution, double minElevation, double maxElevation) {
    mapSizeX_ = mapSizeX;
    mapSizeY_ = mapSizeY;
    robotPositionX_ = robotPositionX;
    robotPositionY_ = robotPositionY;
    mapResolution_ = mapResolution;
    minElevation_ = minElevation;
    maxElevation_ = maxElevation;

    gridMap_.setGeometry(grid_map::Length(mapSizeX_, mapSizeY_), mapResolution_,
            grid_map::Position(robotPositionX_, robotPositionY_));

    return true;
}

void Map::translate(const Eigen::Vector3d& translation) {
    grid_map::Position positionXY(translation.x(), translation.y());

    gridMap_.move(positionXY);
}

}  // namespace ga_slam

