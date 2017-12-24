#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <vector>

namespace ga_slam {

struct Particle {
    double x;
    double y;
    double yaw;

    double weight = 0.;
};

class ParticleFilter {
  public:
    ParticleFilter(void) : numParticles_(0) {}

    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

    void setParameters(int numParticles);

    void initialize(const Pose& initialPose = Pose::Identity()) {}

    void predict(const Pose& deltaPose) {}

    void update(
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud) {}

    void resample(void) {}

    Pose getEstimate(void) const { return pose_; }

  protected:
    std::vector<Particle> particles_;

    int numParticles_;
};

}  // namespace ga_slam

