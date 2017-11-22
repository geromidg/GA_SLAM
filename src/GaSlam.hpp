#ifndef _GASLAM_HPP_
#define _GASLAM_HPP_

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "grid_map_core/grid_map_core.hpp"

using Pose = Eigen::Affine3d;
using Transformation = Eigen::Transform<double, 3, Eigen::Affine>;
using Map = grid_map::GridMap;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

namespace ga_slam {

class GaSlam {
  public:
    GaSlam(void) {}

    explicit GaSlam(const Map& globalMap);

    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    virtual ~GaSlam(void) {}

    void registerData(const Pose& pose, const PointCloudConstPtr& pointCloud);

    void fuseMap(void);

    void correctPose(void);

    const Map& getFusedMap(void) const { return fusedMap_; }

    const Map& getGlobalMap(void) const { return globalMap_; }

    const Pose& getCorrectedPose(void) const { return pose_; }

  protected:
    void translateMap(const Pose& deltaPose);

    void updateMap(const PointCloudConstPtr& pointCloud);

  protected:
    Map map_;
    Map fusedMap_;
    Map globalMap_;

    Pose pose_;
};

}  // namespace ga_slam

#endif  // _GASLAM_HPP_

