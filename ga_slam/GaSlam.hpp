#pragma once

// GA SLAM
#include "ga_slam/TypeDefs.hpp"
#include "ga_slam/mapping/Map.hpp"
#include "ga_slam/mapping/DataRegistration.hpp"
#include "ga_slam/mapping/DataFusion.hpp"
#include "ga_slam/localization/PoseEstimation.hpp"
#include "ga_slam/localization/PoseCorrection.hpp"

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// STL
#include <atomic>
#include <mutex>
#include <chrono>
#include <future>

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

    std::mutex& getGlobalMapMutex(void) {
        return poseCorrection_.getGlobalMapMutex(); }

    void configure(
            double mapLengthX, double mapLengthY, double mapResolution,
            double minElevation, double maxElevation, double voxelSize,
            int numParticles, int resampleFrequency,
            double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
            double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
            double traversedDistanceThreshold, double minSlopeThreshold,
            double slopeSumThresholdMultiplier);

    void poseCallback(const Pose& poseGuess, const Pose& bodyToGroundTF);

    void cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToBodyTF);

    void registerOrbiterCloud(const Cloud::ConstPtr& cloud);

    template<typename T>
    bool isFutureReady(const std::future<T>& future) const {
        if (!future.valid()) return true;
        return (future.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready);
    }

  protected:
    PoseEstimation poseEstimation_;
    PoseCorrection poseCorrection_;
    DataRegistration dataRegistration_;
    DataFusion dataFusion_;

    std::future<void> filterPoseFuture_;
    std::future<void> poseCorrectionFuture_;

    std::atomic<bool> poseInitialized_;

    double voxelSize_;
};

}  // namespace ga_slam

