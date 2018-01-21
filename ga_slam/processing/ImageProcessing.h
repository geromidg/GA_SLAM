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

/** TODO
  */
class ImageProcessing {
  public:
    /// TODO
    ImageProcessing(void) = delete;

    /** TODO
      * @param[in] map TODO
      * @param[out] image TODO
      */
    static void convertMapToImage(const Map& map, Image& image);

    /** TODO
      * @param[in] image TODO
      * @param[in] windowName TODO
      * @param[in] zoom TODO
      */
    static void displayImage(
            const Image& image,
            const std::string& windowName = "Image Display",
            double zoom = 1.);

    /** TODO
      * @param[in] image TODO
      * @param[in] windowName TODO
      * @param[in] width TODO
      * @param[in] height TODO
      */
    static void displayImage(
            const Image& image,
            const std::string& windowName,
            int width,
            int height);

    /** TODO
      * @param[in] inputImage TODO
      * @param[out] outputImage TODO
      * @param[in] useSobel TODO
      * @param[in] sobelKernelSize TODO
      * @param[in] approximate TODO
      */
    static void calculateGradientImage(
            const Image& inputImage,
            Image& outputImage,
            bool useSobel = true,
            int sobelKernelSize = 3,
            bool approximate = false);

    /** TODO
      * @param[in] inputImage TODO
      * @param[out] outputImage TODO
      * @param[in] laplacianKernelSize TODO
      * @param[in] applyGaussianBlur TODO
      * @param[in] gaussianKernelSize TODO
      */
    static void calculateLaplacianImage(
            const Image& inputImage,
            Image& outputImage,
            int laplacianKernelSize = 1,
            bool applyGaussianBlur = false,
            int gaussianKernelSize = 3);

    /** TODO
      * @param[in] originalImage TODO
      * @param[in] templateImage TODO
      * @param[out] matchedPosition TODO
      * @param[in] matchAcceptanceThreshold TODO
      * @param[in] matchImageGradients TODO
      * @param[in] useCrossCorrelation TODO
      * @param[in] displayImage TODO
      * @return TODO
      */
    static bool findBestMatch(
            const Image& originalImage,
            const Image& templateImage,
            cv::Point2d& matchedPosition,
            double matchAcceptanceThreshold,
            bool matchImageGradients = true,
            bool useCrossCorrelation = false,
            bool displayMatch = true);

    /** TODO
      * @param[in] originalImage TODO
      * @param[in] templateImage TODO
      * @param[in] resultImage TODO
      * @param[in] matchedPosition TODO
      * @param[in] zoom TODO
      */
    static void displayMatchedPosition(
            const Image& originalImage,
            const Image& templateImage,
            const Image& resultImage,
            const cv::Point2d& matchedPosition,
            double zoom = 4.);

    /** TODO
      * @param[out] imagePosition TODO
      * @param[in] image TODO
      * @param[in] mapResolution TODO
      */
    static void convertPositionToMapCoordinates(
        cv::Point2d& imagePosition,
        const Image& image,
        double mapResolution);
};

}  // namespace ga_slam

