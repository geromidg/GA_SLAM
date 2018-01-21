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

#include "ga_slam/localization/PoseCorrection.h"

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"
#include "ga_slam/processing/ImageProcessing.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// STL
#include <mutex>
#include <cmath>

namespace ga_slam {

void PoseCorrection::configure(
        double traversedDistanceThreshold,
        double minSlopeThreshold,
        double slopeSumThresholdMultiplier,
        double matchAcceptanceThreshold,
        double globalMapLength,
        double globalMapResolution) {
    traversedDistanceThreshold_ = traversedDistanceThreshold;
    minSlopeThreshold_ = minSlopeThreshold;
    slopeSumThresholdMultiplier_ = slopeSumThresholdMultiplier;
    matchAcceptanceThreshold_ = matchAcceptanceThreshold;

    std::lock_guard<std::mutex> guard(globalMapMutex_);
    globalMap_.setParameters(globalMapLength, globalMapResolution);
}

void PoseCorrection::createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose) {
    std::lock_guard<std::mutex> guard(globalMapMutex_);

    globalMap_.clear();
    globalMap_.translate(Eigen::Vector3d::Zero(), true);

    auto& meanData = globalMap_.getMeanZ();
    auto& varianceData = globalMap_.getVarianceZ();

    size_t cloudIndex = 0;
    size_t mapIndex;

    for (const auto& point : globalCloud->points) {
        cloudIndex++;

        if (!globalMap_.getIndexFromPosition(point.x, point.y, mapIndex))
            continue;

        float& mean = meanData(mapIndex);
        float& variance = varianceData(mapIndex);
        const float& pointVariance = 1.;

        if (!std::isfinite(mean)) {
            mean = point.z;
            variance = pointVariance;
        } else {
            const double innovation = point.z - mean;
            const double gain = variance / (variance + pointVariance);
            mean = mean + (gain * innovation);
            variance = variance * (1. - gain);
        }
    }

    globalMap_.translate(globalCloudPose.translation(), true);
    globalMapPose_ = globalCloudPose;

    globalMap_.setValid(true);
    globalMap_.setTimestamp(globalCloud->header.stamp);

    globalMapInitialized_ = true;
}

bool PoseCorrection::distanceCriterionFulfilled(const Pose& pose) const {
    const Eigen::Vector3d currentXYZ = pose.translation();
    const Eigen::Vector3d lastXYZ = lastCorrectedPose_.translation();
    const Eigen::Vector2d deltaXY = currentXYZ.head(2) - lastXYZ.head(2);

    return deltaXY.norm() >= traversedDistanceThreshold_;
}

bool PoseCorrection::featureCriterionFulfilled(const Map& localMap) const {
    Image image;
    ImageProcessing::convertMapToImage(localMap, image);
    ImageProcessing::calculateGradientImage(image, image);
    cv::threshold(image, image, minSlopeThreshold_, 0., cv::THRESH_TOZERO);

    const double slopeSum = cv::sum(image)[0];
    const double resolution = localMap.getParameters().resolution;
    const double slopeSumThreshold = slopeSumThresholdMultiplier_ / resolution;

    return slopeSum >= slopeSumThreshold;
}

bool PoseCorrection::matchMaps(
        const Map& localMap,
        const Pose& currentPose,
        Pose& correctedPose) {
    if (!globalMapInitialized_) return false;

    Image localImage, globalImage;
    ImageProcessing::convertMapToImage(localMap, localImage);
    const double localMapResolution = localMap.getParameters().resolution;

    std::unique_lock<std::mutex> guard(globalMapMutex_);
    ImageProcessing::convertMapToImage(globalMap_, globalImage);
    const double globalMapResolution = globalMap_.getParameters().resolution;
    guard.unlock();

    const double resolutionRatio = localMapResolution / globalMapResolution;
    cv::resize(localImage, localImage, cv::Size(), resolutionRatio,
            resolutionRatio, cv::INTER_NEAREST);

    cv::Point2d matchedPosition;
    const bool matchFound = ImageProcessing::findBestMatch(globalImage,
            localImage, matchedPosition, matchAcceptanceThreshold_);

    if (matchFound) {
        ImageProcessing::convertPositionToMapCoordinates(matchedPosition,
                globalImage, globalMapResolution);

        const auto newX = globalMapPose_.translation().x() + matchedPosition.x;
        const auto newY = globalMapPose_.translation().y() + matchedPosition.y;
        const auto currentZ = currentPose.translation().z();

        correctedPose = currentPose;
        correctedPose.translation() = Eigen::Vector3d(newX, newY, currentZ);

        lastCorrectedPose_ = correctedPose;
    }

    return matchFound;
}

}  // namespace ga_slam

