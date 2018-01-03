#pragma once

#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/PoseEstimation.hpp"
#include "ga_slam/PoseCorrection.hpp"
#include "ga_slam/DataRegistration.hpp"
#include "ga_slam/DataFusion.hpp"

namespace ga_slam {

class GaSlam {
  public:
    GaSlam(void);

    GaSlam(const GaSlam&) = delete;
    GaSlam& operator=(const GaSlam&) = delete;
    GaSlam(GaSlam&&) = delete;
    GaSlam& operator=(GaSlam&&) = delete;

    const Pose& getPose(void) const { return poseEstimation_.getPose(); }

    const Map& getRawMap(void) const { return dataRegistration_.getMap(); }

    const Map& getFusedMap(void) const { return dataFusion_.getFusedMap(); }

    const Map& getGlobalMap(void) const {
        return poseCorrection_.getGlobalMap(); }

    const Cloud::ConstPtr getProcessedCloud(void) const {
        return processedCloud_; }

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
    void runPoseEstimation(void) {}

    void runPoseCorrection(void) {}

    void runDataRegistration(void) {}

    void runDataFusion(void) {}

  protected:
    PoseEstimation poseEstimation_;
    PoseCorrection poseCorrection_;
    DataRegistration dataRegistration_;
    DataFusion dataFusion_;

    Cloud::Ptr processedCloud_;

    double voxelSize_;
};

}  // namespace ga_slam

