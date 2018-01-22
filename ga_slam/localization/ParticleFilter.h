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

/// Contains a solution of the problem and its likelihood
struct Particle {
    /// 2D position of the robot in space and its orientation
    double x;
    double y;
    double yaw;

    /// Weight representing the likelihood of the particle
    double weight = 0.;
};

/** Implements the particle filter algorithm for estimating the 3-DoF pose
  * (x, y and yaw) of the robot in the continuous space.
  */
class ParticleFilter {
  public:
    ParticleFilter(void) : weightsUpdated_(true) {}

    /// Delete the default copy/move constructors and operators
    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

    /** Sets the parameters used in the filter
      * @param[in] numParticles number of particles used
      * @param[in] initialSigmaX gaussian sigma of x for particle initalization
      * @param[in] initialSigmaY gaussian sigma of y for particle initalization
      * @param[in] initialSigmaYaw gaussian sigma of yaw for particle
      *            initialization
      * @param[in] predictSigmaX gaussian sigma of x for particle prediction
      * @param[in] predictSigmaY gaussian sigma of y for particle prediction
      * @param[in] predictSigmaYaw gaussian sigma of yaw for particle prediction
      */
    void configure(
            int numParticles,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    /** Initializes each particle of the filter by adding gaussian noise to
      * an initial estimate
      * @note the gaussian noise is sampled differently for each particle
      * @param[in] initialX the initial x estimate
      * @param[in] initialY the initial y estimate
      * @param[in] initialYaw the initial yaw estimate
      */
    void initialize(
            double initialX = 0.,
            double initialY = 0.,
            double initialYaw = 0.);

    /** Predicts the state of each particle by adding the input delta values
      * as well as gaussian noise
      * @note noise is added only if an update has happened in the previous
      *       iteration since the frequency of the two steps might differ
      * @param[in] deltaX the delta x estimate
      * @param[in] deltaY the delta y estimate
      * @param[in] deltaYaw the delta yaw estimate
      */
    void predict(
            double deltaX,
            double deltaY,
            double deltaYaw);

    /** Updates the weight of each particle of the filter by transforming a
      * raw point cloud (sensor scan) to the particle's pose and matching it to
      * the map point cloud (cloud converted using the map's elevation values).
      * @note the vector of particles is copied before the matching so the
      *       particles can still be predicted meanwhile
      * @param[in] lastPose the last estimated pose of the filter
      * @param[in] rawCloud the raw point cloud (sensor scan)
      * @param[in] mapCloud the point cloud converted from the local map
      */
    void update(
            const Pose& lastPose,
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

    /** Performs multinomial resampling to the particle population by creating
      * a cummulative distribution of the particles' weights and then sampling
      * from it
      */
    void resample(void);

    /** Calculates the current state estimate of the filter by choosing the
      * best particle
      * @param[out] estimateX the output estimate of x
      * @param[out] estimateY the output estimate of y
      * @param[out] estimateYaw the output estimate of yaw
      */
    void getEstimate(
            double& estimateX,
            double& estimateY,
            double& estimateYaw) const;

  protected:
    /** Returns the particle with highest weight in the population
      * @return the best particle
      */
    Particle getBestParticle(void) const;

    /** Creates a gaussian distribution and samples one value from it
      * @param[in] mean the mean value of the distribution
      * @param[in] sigma the sigma of the distribution
      * @return the sampled value
      */
    double sampleGaussian(double mean, double sigma);

    /** Calclulates the delta pose between a particle and a specific pose
      * @param[in] particle the particle to calculate the delta pose from
      * @param[in] pose the pose reference
      * @return the calculated delta pose
      */
    static Pose getDeltaPoseFromParticle(
            const Particle& particle,
            const Pose& pose);

  protected:
    /// Particle population
    std::vector<Particle> particles_;

    /// Mutex protecting the particles
    mutable std::mutex particlesMutex_;

    /// Whether an update step was executed in the last iteration
    std::atomic<bool> weightsUpdated_;

    /// Random engine generator for sampling from distributions
    std::default_random_engine generator_;

    /// Number of particles in the population
    int numParticles_;

    /// Sigma of gaussian noise added to each particle during initialization
    double initialSigmaX_;
    double initialSigmaY_;
    double initialSigmaYaw_;

    /// Sigma of gaussian noise added to each particle during prediction
    double predictSigmaX_;
    double predictSigmaY_;
    double predictSigmaYaw_;
};

}  // namespace ga_slam

