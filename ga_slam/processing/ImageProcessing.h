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

/** Contains a collection of helper function that are used to process an
  * OpenCV image (mat) or convert it to different data types.
  */
class ImageProcessing {
  public:
    /// Delete the default constructor
    ImageProcessing(void) = delete;

    /** Converts an elevation map to an image of float type
      * @param[in] map the elevation map to be converted
      * @param[out] image the converted image
      */
    static void convertMapToImage(const Map& map, Image& image);

    /** Converts the zoom to width and height and calls the respective
      * overloaded function
      * @param[in] image the image to be displayed
      * @param[in] windowName the name of the display window
      * @param[in] zoom the zoom of the display
      */
    static void displayImage(
            const Image& image,
            const std::string& windowName = "Image Display",
            double zoom = 1.);

    /** Normalizes and displays a float image using OpenCV's GUI
      * @param[in] image the image to be displayed
      * @param[in] windowName the name of the display window
      * @param[in] width the width of the display window
      * @param[in] height the height of the display window
      */
    static void displayImage(
            const Image& image,
            const std::string& windowName,
            int width,
            int height);

    /** Computes the gradients of an image in the x and y axes using the
      * Sobel or Scharr operators and then calculates the absolute or
      * approximate magnitude
      * @param[in] inputImage the input image
      * @param[out] outputImage the gradient image
      * @param[in] useSobel whether to use the Sobel or Scharr operator
      * @param[in] sobelKernelSize the kernel size of the Sobel
      * @param[in] approximate whether to approximate the magnitude using
      *            weighted addition of the absolute gradients in x and y
      */
    static void calculateGradientImage(
            const Image& inputImage,
            Image& outputImage,
            bool useSobel = true,
            int sobelKernelSize = 3,
            bool approximate = false);

    /** Calculates the laplacian of an image
      * @param[in] inputImage the input image
      * @param[out] outputImage the laplacian image
      * @param[in] laplacianKernelSize the kernel size of the laplacian
      * @param[in] applyGaussianBlur whether to apply guassian blur before the
      *            laplacian calculation
      * @param[in] gaussianKernelSize the kernel size of the laplacian
      */
    static void calculateLaplacianImage(
            const Image& inputImage,
            Image& outputImage,
            int laplacianKernelSize = 1,
            bool applyGaussianBlur = false,
            int gaussianKernelSize = 3);

    /** Find the best match given an source and a template image using
      * template matching. The matching can be done using the images as they are
      * or using their gradients
      * @param[in] sourceImage the source image (search space)
      * @param[in] templateImage the image to be matched
      * @param[out] matchedPosition the position of the match
      * @param[in] matchAcceptanceThreshold the minimum score the matched
      *            position must have, in order for the matching to be accepted
      * @param[in] matchImageGradients whether to match the images' gradients
      * @param[in] displayImage whether to display the found match
      * @return true if a match was found
      */
    static bool findBestMatch(
            const Image& sourceImage,
            const Image& templateImage,
            cv::Point2d& matchedPosition,
            double matchAcceptanceThreshold,
            bool matchImageGradients = true,
            bool displayMatch = true);

    /** Displays result of the template matching by drawing rectangles at the
      * matched position in the source and the result images
      * @param[in] sourceImage the source image (search space)
      * @param[in] templateImage the template image that was matched
      * @param[in] resultImage the image representing the normalized result of
      *            the matching for each position
      * @param[in] matchedPosition the position of the match
      * @param[in] zoom the zoom to be applied to the image display
      */
    static void displayMatchedPosition(
            const Image& sourceImage,
            const Image& templateImage,
            const Image& resultImage,
            const cv::Point2d& matchedPosition,
            double zoom = 4.);

    /** Converts a position from image coordinates (origin at top left) to
      * map coordinates (origin at center)
      * @param[in/out] imagePosition the position to be converted
      * @param[in] image the image in which the position corresponds to
      * @param[in] mapResolution the resolution of the map
      */
    static void convertPositionToMapCoordinates(
        cv::Point2d& imagePosition,
        const Image& image,
        double mapResolution);

    /** Replaces the NaN values of an image with 0
      * @param[in] image the image to be processed
      */
    static void replaceNanWithZero(Image& image);
};

}  // namespace ga_slam

