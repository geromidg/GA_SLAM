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

#include "ga_slam/processing/ImageProcessing.h"

// GA SLAM
#include "ga_slam/TypeDefs.h"
#include "ga_slam/mapping/Map.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <string>
#include <cmath>

namespace ga_slam {

void ImageProcessing::convertMapToImage(const Map& map, Image& image) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();

    image = Image::zeros(params.size, params.size, CV_32F);

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const Eigen::Array2i mapIndex(*it);
        const float value = meanData(mapIndex(0), mapIndex(1));

        const Eigen::Array2i index(it.getUnwrappedIndex());
        image.at<float>(index(0), index(1)) = value;
    }
}

void ImageProcessing::displayImage(
        const Image& image,
        const std::string& windowName,
        double zoom) {
    displayImage(image, windowName,
            std::round(zoom * image.cols),
            std::round(zoom * image.rows));
}

void ImageProcessing::displayImage(
        const Image& image,
        const std::string& windowName,
        int width,
        int height) {
    constexpr int waitTimeout = 100;

    Image normalizedImage;
    cv::normalize(image, normalizedImage, 0., 1., cv::NORM_MINMAX);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, width, height);
    cv::imshow(windowName, normalizedImage);
    cv::waitKey(waitTimeout);
}

void ImageProcessing::calculateGradientImage(
        const Image& inputImage,
        Image& outputImage,
        bool useSobel,
        int sobelKernelSize,
        bool approximate) {
    Image gradientX, gradientY;

    if (useSobel) {
        cv::Sobel(inputImage, gradientX, CV_32F, 1, 0, sobelKernelSize);
        cv::Sobel(inputImage, gradientY, CV_32F, 0, 1, sobelKernelSize);
    } else {
        cv::Scharr(inputImage, gradientX, CV_32F, 1, 0);
        cv::Scharr(inputImage, gradientY, CV_32F, 0, 1);
    }

    if (approximate) {
        cv::abs(gradientX);
        cv::abs(gradientY);
        cv::addWeighted(gradientX, 0.5, gradientY, 0.5, 0., outputImage);
    } else {
        cv::magnitude(gradientX, gradientY, outputImage);
    }
}

void ImageProcessing::calculateLaplacianImage(
        const Image& inputImage,
        Image& outputImage,
        int laplacianKernelSize,
        bool applyGaussianBlur,
        int gaussianKernelSize) {
    if (applyGaussianBlur) {
        cv::GaussianBlur(inputImage, outputImage,
                cv::Size(gaussianKernelSize, gaussianKernelSize), 0, 0);
        cv::Laplacian(outputImage, outputImage, CV_32F, laplacianKernelSize);
        cv::abs(outputImage);
    } else {
        cv::Laplacian(inputImage, outputImage, CV_32F, laplacianKernelSize);
    }
}

bool ImageProcessing::findBestMatch(
        const Image& sourceImage,
        const Image& templateImage,
        cv::Point3d& matchedPosition,
        double matchAcceptanceThreshold,
        double matchYawRange,
        double matchYawStep,
        bool matchImageGradients,
        bool displayMatch) {
    Image sourceInput, templateInput;

    if (matchImageGradients) {
        calculateGradientImage(sourceImage, sourceInput);
        calculateGradientImage(templateImage, templateInput);
    } else {
        sourceInput = sourceImage;
        templateInput = templateImage;
    }

    replaceNanWithZero(sourceInput);
    replaceNanWithZero(templateInput);

    constexpr int method = CV_TM_CCORR_NORMED;

    Image warpedTemplate, resultMatrix, bestResultMatrix;
    cv::Point3d bestPosition;
    cv::Point2i maxPosition;
    double maxValue;
    double bestValue = 0.;

    for (double yaw = - matchYawRange / 2.; yaw <= matchYawRange / 2;
            yaw += matchYawStep) {
        warpImage(templateInput, warpedTemplate, yaw);

        cv::matchTemplate(sourceInput, warpedTemplate, resultMatrix, method);
        cv::minMaxLoc(resultMatrix, nullptr, &maxValue, nullptr, &maxPosition);

        if (maxValue > bestValue) {
            bestPosition = cv::Point3d(maxPosition.x, maxPosition.y, yaw);
            bestResultMatrix = resultMatrix;
        }
    }

    const bool matchFound = maxValue > matchAcceptanceThreshold;
    if (matchFound) matchedPosition = bestPosition;

    if (displayMatch && matchFound)
        displayMatchedPosition(sourceInput, warpedTemplate, bestResultMatrix,
                cv::Point2d(matchedPosition.x, matchedPosition.y));

    return matchFound;
}

void ImageProcessing::displayMatchedPosition(
        const Image& sourceImage,
        const Image& templateImage,
        const Image& resultImage,
        const cv::Point2d& matchedPosition,
        double zoom) {
    Image sourceImageClone = sourceImage.clone();
    Image templateImageClone = templateImage.clone();
    Image resultImageClone = resultImage.clone();

    const auto matchedDiagonal = cv::Point2i(
            std::round(matchedPosition.x) + templateImageClone.cols,
            std::round(matchedPosition.y) + templateImageClone.rows);

    const auto color = cv::Scalar::all(0.);
    cv::rectangle(sourceImageClone, matchedPosition, matchedDiagonal, color);
    cv::rectangle(resultImageClone, matchedPosition, matchedDiagonal, color);

    displayImage(sourceImageClone, "Source Image", zoom);
    displayImage(templateImageClone, "Template Image", zoom);
    displayImage(resultImageClone, "Result Image", zoom);
}

void ImageProcessing::convertPositionToMapCoordinates(
        cv::Point3d& imagePosition,
        const Image& image,
        double mapResolution) {
    double mapPositionX = std::round(image.rows / 2.) - imagePosition.y;
    double mapPositionY = std::round(image.cols / 2.) - imagePosition.x;

    imagePosition.x = mapPositionX * mapResolution;
    imagePosition.y = mapPositionY * mapResolution;
}

void ImageProcessing::replaceNanWithZero(Image& image) {
    image.setTo(0., image != image);
}

void ImageProcessing::warpImage(
        const Image& inputImage,
        Image& outputImage,
        double angle) {
    const cv::Point2f center(inputImage.cols / 2.f, inputImage.rows / 2.f);
    Image rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.);

    cv::warpAffine(inputImage, outputImage, rotationMatrix, inputImage.size());
}

}  // namespace ga_slam

