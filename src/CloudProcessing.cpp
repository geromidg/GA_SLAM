#include "ga_slam/CloudProcessing.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

namespace ga_slam {

void CloudProcessing::processCloud(
        const Cloud::ConstPtr& inputCloud,
        Cloud::Ptr& outputCloud,
        std::vector<float>& cloudVariances,
        const Pose& sensorToMapTF,
        const MapParameters& mapParameters,
        double voxelSize) {
    downsampleCloud(inputCloud, outputCloud, voxelSize);
    transformCloudToMap(outputCloud, sensorToMapTF);
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
    pcl::transformPointCloud(*cloud, *cloud, tf);
}

void CloudProcessing::cropCloudToMap(
        Cloud::Ptr& cloud,
        const MapParameters& params) {
    const float pointX = (params.lengthX / 2) + params.positionX;
    const float pointY = (params.lengthY / 2) + params.positionY;

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

    cloud->reserve(params.sizeX * params.sizeY);
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

