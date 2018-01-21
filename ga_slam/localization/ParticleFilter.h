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

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <vector>
#include <random>
#include <mutex>
#include <atomic>

namespace ga_slam {

/** TODO
  */
struct Particle {
    /// TODO
    double x;
    double y;
    double yaw;

    /// TODO
    double weight = 0.;
};

/** TODO
  */
class ParticleFilter {
  public:
    /// TODO
    ParticleFilter(void) : weightsUpdated_(true) {}

    /// TODO
    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

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
            int numParticles,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    /** TODO
      * @param[in] initialX TODO
      * @param[in] initialY TODO
      * @param[in] initialYaw TODO
      */
    void initialize(
            double initialX = 0.,
            double initialY = 0.,
            double initialYaw = 0.);

    /** TODO
      * @param[in] deltaX TODO
      * @param[in] deltaY TODO
      * @param[in] deltaYaw TODO
      */
    void predict(
            double deltaX,
            double deltaY,
            double deltaYaw);

    /** TODO
      * @param[in] lastPose TODO
      * @param[in] rawCloud TODO
      * @param[in] mapCloud TODO
      */
    void update(
            const Pose& lastPose,
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

    /** TODO
      */
    void resample(void);

    /** TODO
      * @param[out] estimateX TODO
      * @param[out] estimateY TODO
      * @param[out] estimateYaw TODO
      */
    void getEstimate(
            double& estimateX,
            double& estimateY,
            double& estimateYaw) const;

  protected:
    /** TODO
      * @param[in] mean TODO
      * @param[in] sigma TODO
      * @return TODO
      */
    double sampleGaussian(double mean, double sigma);

    /** TODO
      * @return TODO
      */
    Particle getBestParticle(void) const;

    /** TODO
      * @param[in] particle TODO
      * @param[in] pose TODO
      * @return TODO
      */
    static Pose getDeltaPoseFromParticle(
            const Particle& particle,
            const Pose& pose);

  protected:
    /// TODO
    std::vector<Particle> particles_;

    /// TODO
    mutable std::mutex particlesMutex_;

    /// TODO
    std::atomic<bool> weightsUpdated_;

    /// TODO
    std::default_random_engine generator_;

    /// TODO
    int numParticles_;

    /// TODO
    double initialSigmaX_;
    double initialSigmaY_;
    double initialSigmaYaw_;

    /// TODO
    double predictSigmaX_;
    double predictSigmaY_;
    double predictSigmaYaw_;
};

}  // namespace ga_slam

