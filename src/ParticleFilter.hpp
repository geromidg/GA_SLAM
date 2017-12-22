#pragma once

#include "ga_slam/TypeDefs.hpp"

#include <vector>

namespace ga_slam {

struct Particle {
    int id;

    double x;
    double y;
    double yaw;

    double weight;
};

class ParticleFilter {
  public:
    explicit ParticleFilter(int numParticles)
            : numParticles_(numParticles) {}

    ~ParticleFilter(void) = delete;
    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;
    ParticleFilter(ParticleFilter&&) = delete;
    ParticleFilter& operator=(ParticleFilter&&) = delete;

    void initialize(const Pose& initialPose);

    void predict(const Pose& deltaPose);

    void update(
            const Cloud::ConstPtr& rawCloud,
            const Cloud::ConstPtr& mapCloud);

    void resample(void);

    Pose getEstimate(void) const;

  protected:
      int numParticles_;

      std::vector<Particle> particles_;
};

}  // namespace ga_slam

