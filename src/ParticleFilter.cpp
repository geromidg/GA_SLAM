#include "ga_slam/ParticleFilter.hpp"

#include "ga_slam/CloudProcessing.hpp"

#include <pcl/common/transforms.h>

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
        const Pose& lastPose,
        const Cloud::ConstPtr& rawCloud,
        const Cloud::ConstPtr& mapCloud) {
    if (mapCloud->empty()) return;

    for (auto& particle : particles_) {
        Cloud::Ptr particleCloud(new Cloud);
        const auto& deltaPose = getDeltaPoseFromParticle(particle, lastPose);
        pcl::transformPointCloud(*mapCloud, *particleCloud, deltaPose);
        double score = CloudProcessing::matchClouds(rawCloud, particleCloud);

        if (score == 0.) score = std::numeric_limits<double>::min();
        particle.weight = 1. / score;
    }
}

void ParticleFilter::resample(void) {
    std::vector<Particle> newParticles;
    std::vector<double> weights;

    std::transform(
            particles_.begin(), particles_.end(), std::back_inserter(weights),
            [] (Particle particle) -> double { return particle.weight; });

    std::discrete_distribution<> distribution(weights.begin(), weights.end());

    for (auto i = 0; i < numParticles_; i++) {
        const auto& newParticle = particles_[distribution(generator_)];
        newParticles.push_back(newParticle);
    }

    particles_ = newParticles;
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
    std::normal_distribution<> distribution(mean, sigma);

    return distribution(generator_);
}

Pose ParticleFilter::getDeltaPoseFromParticle(
        const Particle& particle,
        const Pose& pose) {
    Pose deltaPose;

    deltaPose = Eigen::Translation3d(
            particle.x - pose.translation().x(),
            particle.y - pose.translation().y(),
            0.);

    deltaPose.rotate(Eigen::AngleAxisd(
            particle.yaw - pose.linear().eulerAngles(2, 1, 0)[0],
            Eigen::Vector3d::UnitZ()));

    return deltaPose;
}

}  // namespace ga_slam

