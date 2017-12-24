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

Pose ParticleFilter::getEstimate(void) const {
    Pose estimate = Pose::Identity();
    Particle bestParticle = getBestParticle();

    estimate.translation().x() = bestParticle.x;
    estimate.translation().y() = bestParticle.y;
    estimate.rotate(Eigen::AngleAxisd(bestParticle.yaw,
                Eigen::Vector3d::UnitZ()));

    return estimate;
}

Particle ParticleFilter::getBestParticle(void) const {
    Particle bestParticle = particles_[0];

    for (const auto& particle : particles_)
        if (particle.weight > bestParticle.weight) bestParticle = particle;

    return bestParticle;
}

double ParticleFilter::sampleGaussian(double mean, double sigma) {
    std::normal_distribution<double> distribution(mean, sigma);

    return distribution(generator_);
}

}  // namespace ga_slam

