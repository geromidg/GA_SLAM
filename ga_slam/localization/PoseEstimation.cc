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

#include "ga_slam/localization/PoseEstimation.h"

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/processing/CloudProcessing.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>
#include <cmath>

namespace ga_slam {

void PoseEstimation::configure(int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    particleFilter_.configure(numParticles, initialSigmaX, initialSigmaY,
            initialSigmaYaw, predictSigmaX, predictSigmaY, predictSigmaYaw);
    particleFilter_.initialize();

    resampleFrequency_ = resampleFrequency;
}

void PoseEstimation::predictPose(const Pose& deltaPose) {
    Eigen::Vector3d deltaTranslation = pose_.linear() * deltaPose.translation();
    const double deltaX = deltaTranslation.x();
    const double deltaY = deltaTranslation.y();
    const double deltaYaw = getAnglesFromPose(deltaPose).z();
    particleFilter_.predict(deltaX, deltaY, deltaYaw);

    Pose initialPoseEstimate = pose_ * deltaPose;
    Eigen::Vector3d translation = initialPoseEstimate.translation();
    Eigen::Vector3d angles = getAnglesFromPose(initialPoseEstimate);
    particleFilter_.getEstimate(translation(0), translation(1), angles(2));

    const Pose finalPoseEstimate = createPose(translation, angles);

    std::lock_guard<std::mutex> guard(poseMutex_);
    pose_ = finalPoseEstimate;
}

void PoseEstimation::filterPose(
        const Cloud::ConstPtr& rawCloud,
        const Cloud::ConstPtr& mapCloud) {
    std::unique_lock<std::mutex> guard(poseMutex_);
    const Pose lastPose = pose_;
    guard.unlock();

    particleFilter_.update(lastPose, rawCloud, mapCloud);

    resampleCounter_++;
    if (resampleCounter_ != resampleFrequency_) return;

    particleFilter_.resample();
    resampleCounter_ = 0;
}

Pose PoseEstimation::createPose(
        const Eigen::Vector3d& translation,
        const Eigen::Vector3d& angles) {
    const auto rotation = Eigen::Quaterniond(
            Eigen::AngleAxisd(angles(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(angles(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitX()));

    Pose pose = Pose::Identity();
    pose.translation() = translation;
    pose.linear() = rotation.toRotationMatrix();

    return pose;
}

Eigen::Vector3d PoseEstimation::getAnglesFromPose(const Pose& pose) {
    double roll = atan2(pose(2, 1), pose(2, 2));
    double pitch = asin(-pose(2, 0));
    double yaw = atan2(pose(1, 0), pose(0, 0));

    return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace ga_slam

