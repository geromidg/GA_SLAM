#pragma once

#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/PoseEstimation.hpp"
#include "ga_slam/PoseCorrection.hpp"
#include "ga_slam/DataRegistration.hpp"
#include "ga_slam/DataFusion.hpp"

#include <atomic>
#include <mutex>

namespace ga_slam {

class GaSlam {
  public:
    GaSlam(void);

    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    Pose getPose(void) const { return poseEstimation_.getPose(); }

    std::mutex& getPoseMutex(void) { return poseEstimation_.getPoseMutex(); }

    const Map& getRawMap(void) const { return dataRegistration_.getMap(); }

    std::mutex& getRawMapMutex(void) { return dataRegistration_.getMapMutex(); }

    const Map& getFusedMap(void) const { return dataFusion_.getFusedMap(); }

    const Map& getGlobalMap(void) const {
        return poseCorrection_.getGlobalMap(); }

    void setParameters(
            double mapLengthX, double mapLengthY, double mapResolution,
            double minElevation, double maxElevation,
            double voxelSize, int numParticles,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw);

    void poseCallback(const Pose& poseGuess, const Pose& bodyToGroundTF);

    void cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToBodyTF);

  protected:
    PoseEstimation poseEstimation_;
    PoseCorrection poseCorrection_;
    DataRegistration dataRegistration_;
    DataFusion dataFusion_;

    std::atomic<bool> poseInitialized_;

    double voxelSize_;
};

}  // namespace ga_slam

