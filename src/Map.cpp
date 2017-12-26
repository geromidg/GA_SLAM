#include "ga_slam/Map.hpp"

namespace ga_slam {

Map::Map(void)
        : valid_(false),
          layerMeanZ_("meanZ"),
          layerVarianceZ_("varZ") {
    gridMap_ = GridMap({layerMeanZ_, layerVarianceZ_});
    gridMap_.setBasicLayers({layerMeanZ_, layerVarianceZ_});
    gridMap_.clearBasic();
    gridMap_.resetTimestamp();
}

void Map::setMapParameters(
        double lengthX, double lengthY, double resolution,
        double minElevation, double maxElevation) {
    minElevation_ = minElevation;
    maxElevation_ = maxElevation;

    gridMap_.setGeometry(grid_map::Length(lengthX, lengthY), resolution,
            grid_map::Position::Zero());
}

bool Map::getIndexFromPosition(
        double positionX,
        double positionY,
        size_t& index) const {
    grid_map::Index arrayIndex;
    grid_map::Position position(positionX, positionY);

    if (!gridMap_.getIndex(position, arrayIndex)) return false;

    index = arrayIndex.x() + arrayIndex.y() * getSizeX();

    return true;
}

void Map::getPointFromArrayIndex(
        const grid_map::Index& arrayIndex,
        const Matrix& layerData,
        Eigen::Vector3d& point) const {
    grid_map::Position position;
    gridMap_.getPosition(arrayIndex, position);

    point = Eigen::Vector3d(position.x(), position.y(),
            layerData(arrayIndex.x(), arrayIndex.y()));
}

void Map::translate(const Eigen::Vector3d& translation) {
    gridMap_.move(grid_map::Position(translation.x(), translation.y()));
}

}  // namespace ga_slam

