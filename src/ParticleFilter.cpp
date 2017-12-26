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
    if (firstIteration_) {
        firstIteration_ = false;
        return;
    }

    Cloud::Ptr particleCloud(new Cloud);
    double lastX = lastPose.translation().x();
    double lastY = lastPose.translation().y();
    double lastYaw = lastPose.linear().eulerAngles(2, 1, 0)[0];
    Pose deltaPose;
    double score;

    for (auto& particle : particles_) {
        deltaPose = Eigen::Translation3d(particle.x - lastX,
                particle.y - lastY, 0.);
        deltaPose.rotate(Eigen::AngleAxisd(particle.yaw - lastYaw,
                Eigen::Vector3d::UnitZ()));
        pcl::transformPointCloud(*mapCloud, *particleCloud, deltaPose);

        score = CloudProcessing::measureCloudAlignment(rawCloud, particleCloud);

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

