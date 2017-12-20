#pragma once

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "grid_map_core/GridMap.hpp"

namespace ga_slam {

using Pose = Eigen::Affine3d;
using Map = grid_map::GridMap;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

}  // namespace ga_slam

