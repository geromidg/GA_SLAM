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

struct Particle {
    double x;
    double y;
    double yaw;

    double weight = 0.;
};

class ParticleFilter {
  public:
    ParticleFilter(void) : weightsUpdated_(true) {}

    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

    void configure(
            int numParticles,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    void initialize(
            double initialX = 0.,
            double initialY = 0.,
            double initialYaw = 0.);

    void predict(
            double deltaX,
            double deltaY,
            double deltaYaw);

    void update(
            const Pose& lastPose,
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

    void resample(void);

    void getEstimate(
            double& estimateX,
            double& estimateY,
            double& estimateYaw) const;

  protected:
    double sampleGaussian(double mean, double sigma);

    Particle getBestParticle(void) const;

    static Pose getDeltaPoseFromParticle(
            const Particle& particle,
            const Pose& pose);

  protected:
    std::vector<Particle> particles_;
    mutable std::mutex particlesMutex_;

    std::atomic<bool> weightsUpdated_;

    std::default_random_engine generator_;

    int numParticles_;
    double initialSigmaX_;
    double initialSigmaY_;
    double initialSigmaYaw_;
    double predictSigmaX_;
    double predictSigmaY_;
    double predictSigmaYaw_;
};

}  // namespace ga_slam

