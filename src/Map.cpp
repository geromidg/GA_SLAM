#include "ga_slam/Map.hpp"

namespace ga_slam {

Map::Map(void) : valid_(false) {
    gridMap_ = GridMap({layerMeanZ_, layerVarianceZ_});
    gridMap_.setBasicLayers({layerMeanZ_, layerVarianceZ_});
    gridMap_.clearBasic();
    gridMap_.resetTimestamp();
}

void Map::setParameters(
        double lengthX, double lengthY, double resolution,
        double minElevation, double maxElevation) {
    minElevation_ = minElevation;
    maxElevation_ = maxElevation;

    gridMap_.setGeometry(grid_map::Length(lengthX, lengthY), resolution,
            grid_map::Position::Zero());
}

MapParameters Map::getParameters(void) const {
    MapParameters params;
    params.lengthX = gridMap_.getLength().x();
    params.lengthY = gridMap_.getLength().y();
    params.sizeX = gridMap_.getSize().x();
    params.sizeY = gridMap_.getSize().y();
    params.positionX = gridMap_.getPosition().x();
    params.positionY = gridMap_.getPosition().y();
    params.resolution = gridMap_.getResolution();
    params.minElevation = minElevation_;
    params.maxElevation = maxElevation_;

    return params;
}

bool Map::getIndexFromPosition(
        double positionX,
        double positionY,
        size_t& index) const {
    grid_map::Index arrayIndex;
    grid_map::Position position(positionX, positionY);

    if (!gridMap_.getIndex(position, arrayIndex)) return false;

    index = arrayIndex.x() + arrayIndex.y() * gridMap_.getSize().x();

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

