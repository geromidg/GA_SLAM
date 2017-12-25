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
        const Map& map,
        double voxelSize) {
    downsampleCloud(inputCloud, outputCloud, voxelSize);
    transformCloudToMap(outputCloud, sensorToMapTF);
    cropCloudToMap(outputCloud, map);
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

void CloudProcessing::cropCloudToMap(Cloud::Ptr& cloud, const Map& map) {
    const float pointX = (map.getLengthX() / 2) + map.getPositionX();
    const float pointY = (map.getLengthY() / 2) + map.getPositionY();

    Eigen::Vector4f minCutoffPoint(-pointX, -pointY, map.getMinElevation(), 0.);
    Eigen::Vector4f maxCutoffPoint(pointX, pointY, map.getMaxElevation(), 0.);

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
    cloud->reserve(map.getSizeX() * map.getSizeY());
    cloud->is_dense = true;
    cloud->header.stamp = map.getTimestamp();

    const auto& meanData = map.getMeanZ();
    Eigen::Vector3d point;

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        map.getPointFromArrayIndex(*it, meanData, point);
        cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
}

double CloudProcessing::measureCloudAlignment(
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

