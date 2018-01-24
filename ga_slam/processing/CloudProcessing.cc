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

#include "ga_slam/processing/CloudProcessing.h"

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

// STL
#include <vector>

namespace ga_slam {

void CloudProcessing::processCloud(
        const Cloud::ConstPtr& inputCloud,
        Cloud::Ptr& outputCloud,
        std::vector<float>& cloudVariances,
        const Pose& mapToSensorTF,
        const MapParameters& mapParameters,
        double voxelSize) {
    downsampleCloud(inputCloud, outputCloud, voxelSize);
    transformCloudToMap(outputCloud, mapToSensorTF);
    cropCloudToMap(outputCloud, mapParameters);
    calculateCloudVariances(outputCloud, cloudVariances);
}

void CloudProcessing::downsampleCloud(
        const Cloud::ConstPtr& inputCloud,
        Cloud::Ptr& outputCloud,
        double voxelSize) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*outputCloud);
}

void CloudProcessing::transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf) {
    if (tf.matrix().isIdentity()) return;

    pcl::transformPointCloud(*cloud, *cloud, tf);
}

void CloudProcessing::cropCloudToMap(
        Cloud::Ptr& cloud,
        const MapParameters& params) {
    const float pointX = (params.length / 2) + params.positionX;
    const float pointY = (params.length / 2) + params.positionY;

    Eigen::Vector4f minCutoffPoint(-pointX, -pointY, params.minElevation, 0.);
    Eigen::Vector4f maxCutoffPoint(pointX, pointY, params.maxElevation, 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*cloud);
}

void CloudProcessing::calculateCloudVariances(
        const Cloud::ConstPtr& cloud,
        std::vector<float>& variances) {
    variances.clear();
    variances.reserve(cloud->size());

    for (const auto& point : cloud->points)
        variances.push_back(1.);
}

void CloudProcessing::convertMapToCloud(const Map& map, Cloud::Ptr& cloud) {
    cloud->clear();

    if (!map.isValid()) return;

    const auto params = map.getParameters();

    cloud->reserve(params.size * params.size);
    cloud->is_dense = true;
    cloud->header.stamp = map.getTimestamp();

    const auto& meanData = map.getMeanZ();
    Eigen::Vector3d point;

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        map.getPointFromArrayIndex(*it, meanData, point);
        cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
}

double CloudProcessing::matchClouds(
        const Cloud::ConstPtr& cloud1,
        const Cloud::ConstPtr& cloud2) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);

    Cloud transformedCloud;
    icp.align(transformedCloud);

    return icp.getFitnessScore();
}

}  // namespace ga_slam

