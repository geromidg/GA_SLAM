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
#include "ga_slam/mapping/DataRegistration.h"
#include "ga_slam/localization/PoseEstimation.h"
#include "ga_slam/localization/PoseCorrection.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <atomic>
#include <mutex>
#include <chrono>
#include <future>

namespace ga_slam {

/** GaSlam represents the main class in the GA SLAM library.
  * The asynchronous inputs of the SLAM pipeline are handled by the interface
  * provided here. The library's main modules for localization and mapping are
  * instantiated and coordinated here.
  */
class GaSlam {
  public:
    /// Instantiates the submodules of localization (PoseEstimation and
    /// PoseCorrection) and mapping (DataRegistration)
    GaSlam(void);

    /// Delete the default copy/move constructors and operators
    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    /// Returns the robot's estimated pose
    Pose getPose(void) const { return poseEstimation_.getPose(); }

    /// Returns the mutex protecting the pose
    std::mutex& getPoseMutex(void) { return poseEstimation_.getPoseMutex(); }

    /// Returns the local elevation map
    const Map& getLocalMap(void) const { return dataRegistration_.getMap(); }

    /// Returns the mutex protecting the local map
    std::mutex& getLocalMapMutex(void) {
        return dataRegistration_.getMapMutex(); }

    /// Returns the global elevation map
    const Map& getGlobalMap(void) const {
        return poseCorrection_.getGlobalMap(); }

    /// Returns the mutex protecting the global elevation map
    std::mutex& getGlobalMapMutex(void) {
        return poseCorrection_.getGlobalMapMutex(); }

    /// Returns the array with the particles from the particle filter
    Eigen::ArrayXXd getParticlesArray(void) const {
        return poseEstimation_.getParticlesArray(); }

    /** Configures the managed submodules by passing the respective parameters
      * @param[in] mapLength size of one dimension of the local map in meters
      * @param[in] mapResolution resolution of each square cell of the local map
      *            in meters
      * @param[in] minElevation minimum cutoff height of the voxelized cloud
      * @param[in] maxElevation maximum cutoff height of the voxelized cloud
      * @param[in] voxelSize dimension of each voxel of the point cloud
      * @param[in] numParticles number of particles used in the particle filter
      * @param[in] resampleFrequency number of iterations of the particle filter
      *            before resampling
      * @param[in] initialSigmaX gaussian sigma of x for particle initialization
      * @param[in] initialSigmaY gaussian sigma of y for particle initialization
      * @param[in] initialSigmaYaw gaussian sigma of yaw for particle
      *            initialization
      * @param[in] predictSigmaX gaussian sigma of x for particle prediction
      * @param[in] predictSigmaY gaussian sigma of y for particle prediction
      * @param[in] predictSigmaYaw gaussian sigma of yaw for particle prediction
      * @param[in] traversedDistanceThreshold distance the robot must traverse
      *            before a new pose correction
      * @param[in] minSlopeThreshold minimum slope (in radians) a cell of the
      *            map needs to have to be considered as an elevation feature
      * @param[in] slopeSumThresholdMultiplier divided by the resolution of the
      *            map to produce the slope sum threshold, which determines if
      *            the map has enough elevation features for map matching
      * @param[in] matchAcceptanceThreshold minimum score the matched position
      *            from template matching must have, in order for the matching
      *            to be accepted
      * @param[in] globalMapLength size of one dimension of the global map
      * @param[in] globalMapResolution resolution of the global map in meters
      */
    void configure(
            double mapLength, double mapResolution,
            double minElevation, double maxElevation, double voxelSize,
            int numParticles, int resampleFrequency,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
            double traversedDistanceThreshold, double minSlopeThreshold,
            double slopeSumThresholdMultiplier, double matchAcceptanceThreshold,
            double globalMapLength, double globalMapResolution);

    /** Handles the input delta pose data from odometry. The delta pose is
      * used to predict the robot's current pose and update the map's position
      * @param[in] odometryDeltaPose the delta pose as received from odometry
      */
    void poseCallback(const Pose& odometryDeltaPose);

    /** Handles the input pose data from the IMU sensor. The orientation
      * part of the pose is fused with the current pose estimate
      * @param[in] imuOrientation the input orientation as received from the IMU
      */
    void imuCallback(const Pose& imuOrientation);

    /** Handles the input point cloud data from various sensors. The cloud is
      * processed and registered to the local map. If possible, scan-to-map and
      * map-to-map matchings are perform to improve the pose and map estimates
      * @param[in] cloud the point cloud as received from a sensor
      * @param[in] bodyToSensorTF the transformation applied to the cloud
      */
    void cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& bodyToSensorTF = Pose::Identity());

    /** Passes the input global cloud and pose to be used in map matching to the
      * respective module
      * @param[in] globalCloud the point cloud as received from the orbiter
      * @param[in] globalCloudPose the pose of point cloud in the world
      */
    void createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose);

  protected:
    /** Converts the local elevation map to a point cloud and performs a
      * scan-to-map matching using the raw (sensor) point cloud for
      * each particle of the particle filter
      * @param[in] rawCloud the raw point cloud to be matched
      */
    void matchLocalMapToRawCloud(const Cloud::ConstPtr& rawCloud);

    /** Checks if the robot has traversed enough distance and if the local map
      * contains enough elevation features and performs a global-to-local map
      * matching to correct the robot's pose estimate
      */
    void matchLocalMapToGlobalMap(void);

    /** Checks if the future is ready to hold a new task's state
      * @param[in] future the future to be checked
      * @return true if ready
      */
    template<typename T>
    bool isFutureReady(const std::future<T>& future) const {
        if (!future.valid()) return true;
        return (future.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready);
    }

  protected:
    /// Instances of the main submodules for localization and mapping
    PoseEstimation poseEstimation_;
    PoseCorrection poseCorrection_;
    DataRegistration dataRegistration_;

    /// Futures to hold the state of the scan-to-map and map-to-map matching
    /// asynchronous tasks
    std::future<void> scanToMapMatchingFuture_;
    std::future<void> mapToMapMatchingFuture_;

    /// Whether a pose has been received yet
    std::atomic<bool> poseInitialized_;

    /// Voxel size of the downsampled point cloud after it it processed
    double voxelSize_;
};

}  // namespace ga_slam

