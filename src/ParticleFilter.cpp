#include "ga_slam/ParticleFilter.hpp"

#include "ga_slam/CloudProcessing.hpp"

#include <random>
#include <limits>

namespace ga_slam {

void ParticleFilter::setParameters(
        int numParticles,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw) {
    numParticles_ = numParticles;
    initialSigmaX_ = initialSigmaX;
    initialSigmaY_ = initialSigmaY;
    initialSigmaYaw_ = initialSigmaYaw;
    predictSigmaX_ = predictSigmaX;
    predictSigmaY_ = predictSigmaY;
    predictSigmaYaw_ = predictSigmaYaw;

    particles_.clear();
    particles_.resize(numParticles_);
}

void ParticleFilter::initialize(
        double initialX,
        double initialY,
        double initialYaw) {
    for (auto& particle : particles_) {
        particle.x = sampleGaussian(initialX, initialSigmaX_);
        particle.y = sampleGaussian(initialY, initialSigmaY_);
        particle.yaw = sampleGaussian(initialYaw, initialSigmaYaw_);
    }
}

void ParticleFilter::predict(
        double deltaX,
        double deltaY,
        double deltaYaw) {
    for (auto& particle : particles_) {
        particle.x = sampleGaussian(particle.x + deltaX, predictSigmaX_);
        particle.y = sampleGaussian(particle.y + deltaY, predictSigmaY_);
        particle.yaw = sampleGaussian(particle.yaw + deltaYaw,
                predictSigmaYaw_);
    }
}

void ParticleFilter::update(
        const Cloud::ConstPtr& rawCloud,
        const Cloud::ConstPtr& mapCloud) {
    if (firstIteration_) {
        firstIteration_ = false;
        return;
    }

    for (auto& particle : particles_) {
        double score = CloudProcessing::measureCloudAlignment(
                rawCloud, mapCloud);

        if (score == 0.) score = std::numeric_limits<double>::min();

        particle.weight = 1. / score;
    }
}

void ParticleFilter::getEstimate(
        double& estimateX,
        double& estimateY,
        double& estimateYaw) const {
    const auto& bestParticle = getBestParticle();

    estimateX = bestParticle.x;
    estimateY = bestParticle.y;
    estimateYaw = bestParticle.yaw;
}

Particle ParticleFilter::getBestParticle(void) const {
    auto bestParticle = particles_[0];

    for (const auto& particle : particles_)
        if (particle.weight > bestParticle.weight) bestParticle = particle;

    return bestParticle;
}

double ParticleFilter::sampleGaussian(double mean, double sigma) {
    std::normal_distribution<double> distribution(mean, sigma);

    return distribution(generator_);
}

}  // namespace ga_slam

