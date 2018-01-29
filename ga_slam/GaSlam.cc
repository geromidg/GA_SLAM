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

#include "ga_slam/GaSlam.h"

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/processing/CloudProcessing.h"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>
#include <future>

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_(),
          poseInitialized_(false) {
}

void GaSlam::configure(
        double mapLength, double mapResolution,
        double minElevation, double maxElevation, double voxelSize,
        int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
        double traversedDistanceThreshold, double minSlopeThreshold,
        double slopeSumThresholdMultiplier, double matchAcceptanceThreshold,
        double globalMapLength, double globalMapResolution) {
    voxelSize_ = voxelSize;

    poseEstimation_.configure(numParticles, resampleFrequency,
            initialSigmaX, initialSigmaY, initialSigmaYaw,
            predictSigmaX, predictSigmaY, predictSigmaYaw);

    poseCorrection_.configure(traversedDistanceThreshold, minSlopeThreshold,
            slopeSumThresholdMultiplier, matchAcceptanceThreshold,
            globalMapLength, globalMapResolution);

    dataRegistration_.configure(mapLength, mapResolution, minElevation,
            maxElevation);
}

void GaSlam::poseCallback(const Pose& odometryDeltaPose) {
    if (!poseInitialized_) poseInitialized_ = true;

    poseEstimation_.predictPose(odometryDeltaPose);
    dataRegistration_.translateMap(getPose());
}

void GaSlam::imuCallback(const Pose& imuOrientation) {
    if (!poseInitialized_) return;

    poseEstimation_.fuseImuOrientation(imuOrientation);
}

void GaSlam::cloudCallback(
        const Cloud::ConstPtr& cloud,
        const Pose& bodyToSensorTF) {
    if (!poseInitialized_) return;

    const auto mapToSensorTF = getPose() * bodyToSensorTF;
    const auto mapParameters = getRawMap().getParameters();
    Cloud::Ptr processedCloud(new Cloud);
    std::vector<float> cloudVariances;

    CloudProcessing::processCloud(cloud, processedCloud, cloudVariances,
            mapToSensorTF, mapParameters, voxelSize_);

    dataRegistration_.updateMap(processedCloud, cloudVariances);

    if (isFutureReady(scanToMapMatchingFuture_))
        scanToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToRawCloud, this, processedCloud);

    if (isFutureReady(mapToMapMatchingFuture_))
        mapToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToGlobalMap, this);
}

void GaSlam::createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose) {
    poseCorrection_.createGlobalMap(globalCloud, globalCloudPose);
}

void GaSlam::matchLocalMapToRawCloud(const Cloud::ConstPtr& rawCloud) {
    Cloud::Ptr mapCloud(new Cloud);

    std::unique_lock<std::mutex> guard(getRawMapMutex());
    const auto& map = getRawMap();
    CloudProcessing::convertMapToCloud(map, mapCloud);
    guard.unlock();

    poseEstimation_.filterPose(rawCloud, mapCloud);
}

void GaSlam::matchLocalMapToGlobalMap(void) {
    const auto currentPose = getPose();
    if (!poseCorrection_.distanceCriterionFulfilled(currentPose)) return;

    std::unique_lock<std::mutex> guard(getRawMapMutex());
    const auto& map = getRawMap();
    if (!poseCorrection_.featureCriterionFulfilled(map)) return;

    Pose correctionDeltaPose;
    const bool matchFound = poseCorrection_.matchMaps(map, currentPose,
            correctionDeltaPose);
    guard.unlock();

    if (matchFound) poseEstimation_.predictPose(correctionDeltaPose);
}

}  // namespace ga_slam

