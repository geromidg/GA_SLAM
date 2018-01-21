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

#include "ga_slam/mapping/Map.h"

// Grid Map
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/GridMap.hpp"

// Eigen
#include <Eigen/Core>

namespace ga_slam {

Map::Map(void) : valid_(false) {
    gridMap_ = GridMap({layerMeanZ_, layerVarianceZ_});
    gridMap_.setBasicLayers({layerMeanZ_, layerVarianceZ_});
    gridMap_.clearBasic();
    gridMap_.resetTimestamp();
}

void Map::setParameters(
        double length,
        double resolution,
        double minElevation,
        double maxElevation) {
    minElevation_ = minElevation;
    maxElevation_ = maxElevation;

    gridMap_.setGeometry(grid_map::Length(length, length), resolution,
            grid_map::Position::Zero());
}

MapParameters Map::getParameters(void) const {
    MapParameters params;
    params.length = gridMap_.getLength().x();
    params.size = gridMap_.getSize().x();
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

void Map::translate(const Eigen::Vector3d& translation, bool moveData) {
    const auto newPosition = grid_map::Position(translation.x(),
            translation.y());

    if (moveData)
        gridMap_.setPosition(newPosition);
    else
        gridMap_.move(newPosition);
}

}  // namespace ga_slam

