#pragma once

#include "ga_slam/Map.hpp"

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ga_slam {

using Pose = Eigen::Affine3d;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

}  // namespace ga_slam

