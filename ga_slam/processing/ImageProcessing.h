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

// OpenCV
#include <opencv2/core/core.hpp>

// STL
#include <string>

namespace ga_slam {

class ImageProcessing {
  public:
    ImageProcessing(void) = delete;

    static void convertMapToImage(const Map& map, Image& image);

    static void displayImage(
            const Image& image,
            const std::string& windowName = "Image Display",
            double zoom = 1.);

    static void displayImage(
            const Image& image,
            const std::string& windowName,
            int width,
            int height);

    static void calculateGradientImage(
            const Image& inputImage,
            Image& outputImage,
            bool useSobel = true,
            int sobelKernelSize = 3,
            bool approximate = false);

    static void calculateLaplacianImage(
            const Image& inputImage,
            Image& outputImage,
            int laplacianKernelSize = 1,
            bool applyGaussianBlur = false,
            int gaussianKernelSize = 3);

    static bool findBestMatch(
            const Image& originalImage,
            const Image& templateImage,
            cv::Point2d& matchedPosition,
            double matchAcceptanceThreshold,
            bool matchImageGradients = true,
            bool useCrossCorrelation = false,
            bool displayMatch = true);

    static void displayMatchedPosition(
            const Image& originalImage,
            const Image& templateImage,
            const Image& resultImage,
            const cv::Point2d& matchedPosition,
            double zoom = 4.);

    static void convertPositionToMapCoordinates(
        cv::Point2d& imagePosition,
        const Image& image,
        double mapResolution);
};

}  // namespace ga_slam

