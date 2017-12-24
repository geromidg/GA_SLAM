#include "ga_slam/ParticleFilter.hpp"

#include <random>

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
    predictSigmaYaw_ = initialSigmaYaw;

    particles_.clear();
    particles_.resize(numParticles_);
}

void ParticleFilter::initialize(const Pose& initialPose) {
    double initialX = initialPose.translation().x();
    double initialY = initialPose.translation().y();
    double initialYaw = initialPose.linear().eulerAngles(2, 1, 0)[0];

    for (auto& particle : particles_) {
        particle.x = sampleGaussian(initialX, initialSigmaX_);
        particle.y = sampleGaussian(initialY, initialSigmaY_);
        particle.yaw = sampleGaussian(initialYaw, initialSigmaYaw_);
    }
}

void ParticleFilter::predict(const Pose& deltaPose) {
    double deltaX = deltaPose.translation().x();
    double deltaY = deltaPose.translation().y();
    double deltaYaw = deltaPose.linear().eulerAngles(2, 1, 0)[0];

    for (auto& particle : particles_) {
        particle.x = sampleGaussian(particle.x + deltaX, predictSigmaX_);
        particle.y = sampleGaussian(particle.y + deltaY, predictSigmaY_);
        particle.yaw = sampleGaussian(particle.yaw + deltaYaw,
                predictSigmaYaw_);
    }
}

double ParticleFilter::sampleGaussian(double mean, double sigma) {
    std::normal_distribution<double> distribution(mean, sigma);

    return distribution(generator_);
}

}  // namespace ga_slam

