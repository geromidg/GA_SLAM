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
#include <vector>

namespace ga_slam {

void PoseCorrection::configure(
        double traversedDistanceThreshold,
        double minSlopeThreshold,
        double slopeSumThresholdMultiplier,
        double matchAcceptanceThreshold,
        bool matchYaw,
        double matchYawRange,
        double matchYawStep,
        double globalMapLength,
        double globalMapResolution) {
    traversedDistanceThreshold_ = traversedDistanceThreshold;
    minSlopeThreshold_ = minSlopeThreshold;
    slopeSumThresholdMultiplier_ = slopeSumThresholdMultiplier;
    matchAcceptanceThreshold_ = matchAcceptanceThreshold;
    matchYaw_ = matchYaw;
    matchYawRange_ = matchYawRange;
    matchYawStep_ = matchYawStep;

    globalDataRegistration_.configure(globalMapLength, globalMapResolution);
}

void PoseCorrection::createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose) {
    constexpr float globalCloudVariance = 1.f;
    std::vector<float> globalCloudVariances;
    globalCloudVariances.resize(globalCloud->size(), globalCloudVariance);

    globalDataRegistration_.clear();
    globalDataRegistration_.translateMap(Pose::Identity(), true);
    globalDataRegistration_.updateMap(globalCloud, globalCloudVariances);
    globalDataRegistration_.translateMap(globalCloudPose, true);

    globalMapPose_ = globalCloudPose;
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
        Pose& correctionDeltaPose) {
    if (!globalMapInitialized_) return false;

    Image localImage, globalImage;
    ImageProcessing::convertMapToImage(localMap, localImage);
    const double localMapResolution = localMap.getParameters().resolution;

    std::unique_lock<std::mutex> guard(getGlobalMapMutex());
    const auto& globalMap = getGlobalMap();
    ImageProcessing::convertMapToImage(globalMap, globalImage);
    const double globalMapResolution = globalMap.getParameters().resolution;
    guard.unlock();

    const double resolutionRatio = localMapResolution / globalMapResolution;
    cv::resize(localImage, localImage, cv::Size(), resolutionRatio,
            resolutionRatio, cv::INTER_NEAREST);

    cv::Point3d matchedPosition;
    const bool matchFound = ImageProcessing::findBestMatch(globalImage,
            localImage, matchedPosition, matchAcceptanceThreshold_,
            matchYaw_, matchYawRange_, matchYawStep_);

    if (matchFound) {
        ImageProcessing::convertPositionToMapCoordinates(matchedPosition,
                globalImage, globalMapResolution);

        const Eigen::Vector2d mapXY = globalMapPose_.translation().head(2);
        const Eigen::Vector2d currentXY = currentPose.translation().head(2);
        correctionDeltaPose = Eigen::Translation3d(
                mapXY.x() + matchedPosition.x - currentXY.x(),
                mapXY.y() + matchedPosition.y - currentXY.y(), 0.);

        lastCorrectedPose_ = currentPose * correctionDeltaPose;
    }

    return matchFound;
}

}  // namespace ga_slam

