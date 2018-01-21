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
#include "ga_slam/mapping/DataFusion.h"
#include "ga_slam/localization/PoseEstimation.h"
#include "ga_slam/localization/PoseCorrection.h"

// Eigen
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

/** TODO
  */
class GaSlam {
  public:
    /// TODO
    GaSlam(void);

    /// TODO
    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    /// TODO
    Pose getPose(void) const { return poseEstimation_.getPose(); }

    /// TODO
    std::mutex& getPoseMutex(void) { return poseEstimation_.getPoseMutex(); }

    /// TODO
    const Map& getRawMap(void) const { return dataRegistration_.getMap(); }

    /// TODO
    std::mutex& getRawMapMutex(void) { return dataRegistration_.getMapMutex(); }

    /// TODO
    const Map& getFusedMap(void) const { return dataFusion_.getFusedMap(); }

    /// TODO
    const Map& getGlobalMap(void) const {
        return poseCorrection_.getGlobalMap(); }

    /// TODO
    std::mutex& getGlobalMapMutex(void) {
        return poseCorrection_.getGlobalMapMutex(); }

    /** TODO
      * @param[in] mapLength TODO
      * @param[in] mapResolution TODO
      * @param[in] minElevation TODO
      * @param[in] maxElevation TODO
      * @param[in] voxelSize TODO
      * @param[in] numParticles TODO
      * @param[in] resampleFrequency TODO
      * @param[in] initialSigmaX TODO
      * @param[in] initialSigmaY TODO
      * @param[in] initialSigmaYaw TODO
      * @param[in] predictSigmaX TODO
      * @param[in] predictSigmaY TODO
      * @param[in] predictSigmaYaw TODO
      * @param[in] traversedDistanceThreshold TODO
      * @param[in] minSlopeThreshold TODO
      * @param[in] slopeSumThresholdMultiplier TODO
      * @param[in] matchAcceptanceThreshold TODO
      * @param[in] globalMapLength TODO
      * @param[in] globalMapResolution TODO
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

    /** TODO
      * @param[in] poseGuess TODO
      * @param[in] bodyToGroundTF TODO
      */
    void poseCallback(const Pose& poseGuess, const Pose& bodyToGroundTF);

    /** TODO
      * @param[in] cloud TODO
      * @param[in] sensorToBodyTF TODO
      */
    void cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToBodyTF);

    /** TODO
      * @param[in] rawCloud TODO
      */
    void matchLocalMapToRawCloud(const Cloud::ConstPtr& rawCloud);

    /** TODO
      */
    void matchLocalMapToGlobalMap(void);

    /** TODO
      * @param[in] globalCloud TODO
      * @param[in] globalCloudPose TODO
      */
    void createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose);

    /** TODO
      * @param[in] future TODO
      * @return TODO
      */
    template<typename T>
    bool isFutureReady(const std::future<T>& future) const {
        if (!future.valid()) return true;
        return (future.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready);
    }

  protected:
    /// TODO
    PoseEstimation poseEstimation_;
    PoseCorrection poseCorrection_;
    DataRegistration dataRegistration_;
    DataFusion dataFusion_;

    /// TODO
    std::future<void> scanToMapMatchingFuture_;
    std::future<void> mapToMapMatchingFuture_;

    /// TODO
    std::atomic<bool> poseInitialized_;

    /// TODO
    double voxelSize_;
};

}  // namespace ga_slam

