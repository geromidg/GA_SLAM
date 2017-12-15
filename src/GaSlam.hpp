#ifndef _GASLAM_HPP_
#define _GASLAM_HPP_

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "grid_map_core/grid_map_core.hpp"

using Pose = Eigen::Affine3d;
using Map = grid_map::GridMap;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

namespace ga_slam {

class GaSlam {
  public:
    GaSlam(void) : GaSlam(Map()) {}

    explicit GaSlam(const Map& globalMap);

    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    const Map& getRawMap(void) const { return rawMap_; }

    const Map& getFusedMap(void) const { return fusedMap_; }

    const Map& getGlobalMap(void) const { return globalMap_; }

    const Pose& getCorrectedPose(void) const { return correctedPose_; }

    const PointCloud::ConstPtr getFilteredPointCloud(void) const {
        return filteredCloud_; }

    bool setParameters(
            double mapSizeX, double mapSizeY,
            double robotPositionX, double robotPositionY,
            double mapResolution, double voxelSize,
            double minElevation, double maxElevation);

    void registerData(
            const PointCloud::ConstPtr& inputCloud,
            const Pose& inputPose,
            const Pose& sensorToBodyTF,
            const Pose& bodyToGroundTF = Pose::Identity());

    void fuseMap(void);

    void correctPose(void);

  protected:
    void processPointCloud(
            const PointCloud::ConstPtr& inputCloud,
            const Pose& sensorToMapTF);

    void translateMap(const Eigen::Vector3d& translation);

    void updateMap(void);

    void downsamplePointCloud(const PointCloud::ConstPtr& inputCloud);

    void transformPointCloudToMap(const Pose& sensorToMapTF);

    void cropPointCloudToMap(void);

    void calculatePointCloudVariances(void);

    void convertMapToPointCloud(PointCloud::Ptr cloud) const;

    static double getICPFitnessScore(
            const PointCloud::ConstPtr cloud1,
            const PointCloud::ConstPtr cloud2) {}

    static void fuseGaussians(
            float& mean1, float& variance1,
            const float& mean2, const float& variance2);

  protected:
    Map rawMap_;
    Map fusedMap_;
    Map globalMap_;

    Pose correctedPose_;

    PointCloud::Ptr filteredCloud_;
    std::vector<float> cloudVariances_;

    double mapSizeX_;
    double mapSizeY_;
    double robotPositionX_;
    double robotPositionY_;
    double mapResolution_;
    double voxelSize_;
    double minElevation_;
    double maxElevation_;

    const std::string layerMeanZ_;
    const std::string layerVarZ_;
};

}  // namespace ga_slam

#endif  // _GASLAM_HPP_

