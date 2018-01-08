#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <vector>
#include <random>
#include <mutex>

namespace ga_slam {

struct Particle {
    double x;
    double y;
    double yaw;

    double weight = 0.;
};

class ParticleFilter {
  public:
    ParticleFilter(void) {}

    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

    std::mutex& getParticlesMutex(void) { return particlesMutex_; }

    void setParameters(
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

