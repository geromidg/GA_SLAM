#include "ga_slam/ParticleFilter.hpp"

namespace ga_slam {

void ParticleFilter::setParameters(int numParticles) {
    numParticles_ = numParticles;

    particles_.clear();
    particles_.resize(numParticles_);
}

}  // namespace ga_slam

