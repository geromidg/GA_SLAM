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
#include "ga_slam/localization/ParticleFilter.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <mutex>
#include <atomic>

namespace ga_slam {

/** TODO
  */
class PoseEstimation {
  public:
    /// TODO
    PoseEstimation(void)
            : pose_(Pose::Identity()),
              resampleCounter_(0),
              particleFilter_() {}

    /// TODO
    PoseEstimation(const PoseEstimation&) = delete;
    PoseEstimation& operator=(const PoseEstimation&) = delete;
    PoseEstimation(PoseEstimation&&) = delete;
    PoseEstimation& operator=(PoseEstimation&&) = delete;

    /// TODO
    Pose getPose(void) const {
        std::lock_guard<std::mutex> guard(poseMutex_);
        return pose_;
    }

    /// TODO
    std::mutex& getPoseMutex(void) { return poseMutex_; }

    /** TODO
      * @param[in] numParticles TODO
      * @param[in] initialSigmaX TODO
      * @param[in] initialSigmaY TODO
      * @param[in] initialSigmaYaw TODO
      * @param[in] predictSigmaX TODO
      * @param[in] predictSigmaY TODO
      * @param[in] predictSigmaYaw TODO
      */
    void configure(
            int numParticles, int resampleFrequency,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    /** TODO
      * @param[in] poseGuess TODO
      */
    void predictPose(const Pose& poseGuess = Pose::Identity());

    /** TODO
      * @param[in] rawCloud TODO
      * @param[in] mapCloud TODO
      */
    void filterPose(
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

  protected:
    /** TODO
      * @param[in] translation TODO
      * @param[in] angles TODO
      * @return TODO
      */
    static Pose createPose(
            const Eigen::Vector3d& translation,
            const Eigen::Vector3d& angles);

    /** TODO
      * @param[in] pose TODO
      * @return TODO
      */
    static Eigen::Vector3d getAnglesFromPose(const Pose& pose);

  protected:
    /// TODO
    Pose pose_;

    /// TODO
    mutable std::mutex poseMutex_;

    /// TODO
    std::atomic<int> resampleCounter_;

    /// TODO
    int resampleFrequency_;

    /// TODO
    ParticleFilter particleFilter_;
};

}  // namespace ga_slam

