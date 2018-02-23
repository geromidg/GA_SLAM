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

#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>

namespace ga_slam {

/** Contains a collection of helper functions that are used to process a
  * PCL point cloud or convert it to different data types.
  */
class CloudProcessing {
  public:
    /// Delete the default constructor
    CloudProcessing(void) = delete;

    /** Processes a point cloud and prepares it for registration.
      * The processing steps are: downsample, transformation to map reference
      * and cropping to map's dimensions. In addition, a variance value is
      * computed for each point of the cloud
      * @param[in] inputCloud the point cloud to be processed
      * @param[out] outputCloud the processed point cloud
      * @param[out] cloudVariances the vector of the computed variances
      * @param[in] sensorToMapTF the transformation to be applied to the cloud
      * @param[in] mapParameters the structure containing the map's parameters
      * @param[in] voxelSize the voxel size needed to downsample the cloud
      * @param[in] depthSigmaCoeff1 the first coefficient of the uncertainty
      *            equation of the depth sensor
      * @param[in] depthSigmaCoeff2 the second coefficient of the uncertainty
      *            equation of the depth sensor
      * @param[in] depthSigmaCoeff3 the third coefficient of the uncertainty
      *            equation of the depth sensor
      */
    static void processCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            std::vector<float>& cloudVariances,
            const Pose& robotPose,
            const Pose& sensorToMapTF,
            const MapParameters& mapParameters,
            double voxelSize,
            double depthSigmaCoeff1,
            double depthSigmaCoeff2,
            double depthSigmaCoeff3);

    /** Downsamples a point cloud using a voxel grid to the specified resolution
      * @param[in] inputCloud the point cloud to be downsampled
      * @param[out] outputCloud the downsampled point cloud
      * @param[in] voxelSize the leaf size of each voxel in the voxel grid
      */
    static void downsampleCloud(
            const Cloud::ConstPtr& inputCloud,
            Cloud::Ptr& outputCloud,
            double voxelSize);

    /** Transforms a point cloud to the map reference using an affine transform
      * @param[in/out] cloud the point cloud to be transformed
      * @param[in] tf the transformation to be applied to the point cloud
      */
    static void transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf);

    /** Crops a point cloud to the map's dimensions
      * @param[in/out] cloud the point cloud to be cropped
      * @param[in] mapParameters the structure containing the map's parameters
      */
    static void cropCloudToMap(
            Cloud::Ptr& cloud,
            const Pose& robotPose,
            const MapParameters& mapParameters);

    /** Calculates a variance value for each point of the point cloud using the
      * sensor's model. To make the calculation sensor agnostic, a quadratic
      * equation models the uncertainty depending on the distance of each
      * points
      * @param[in] cloud the input point cloud
      * @param[out] variances the vector of the computed variances
      * @param[in] depthSigmaCoeff1 the first coefficient of the uncertainty
      *            equation of the depth sensor
      * @param[in] depthSigmaCoeff2 the second coefficient of the uncertainty
      *            equation of the depth sensor
      * @param[in] depthSigmaCoeff3 the third coefficient of the uncertainty
      *            equation of the depth sensor
      */
    static void calculateCloudVariances(
            const Cloud::ConstPtr& cloud,
            std::vector<float>& variances,
            double depthSigmaCoeff1,
            double depthSigmaCoeff2,
            double depthSigmaCoeff3);

    /** Converts an elevation map to a non-organized dense point cloud
      * @param[in] map the elevation map to be converted
      * @param[out] cloud the converted point cloud
      */
    static void convertMapToCloud(const Map& map, Cloud::Ptr& cloud);

    /** Matches two clouds by aligning them and measuring the mean square error
      * of their points using one iteration of ICP
      * @param[in] cloud1 the first point cloud
      * @param[in] cloud2 the second point cloud
      * @return the fitness score of the match (zero score means perfect match)
      */
    static double matchClouds(
            const Cloud::ConstPtr& cloud1,
            const Cloud::ConstPtr& cloud2);
};

}  // namespace ga_slam

