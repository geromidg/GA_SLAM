#include "ga_slam/GaSlam.hpp"

#include "ga_slam/CloudProcessing.hpp"

namespace ga_slam {

GaSlam::GaSlam(void)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          dataFusion_() {
    processedCloud_.reset(new Cloud);
}

void GaSlam::setParameters(
        double mapLengthX, double mapLengthY, double mapResolution,
        double minElevation, double maxElevation, double voxelSize) {
    poseEstimation_.setParameters(20, 0., 0., 0., 0.5, 0.5, M_PI/8);

    dataRegistration_.setParameters(mapLengthX, mapLengthY, mapResolution,
            minElevation, maxElevation);

    voxelSize_ = voxelSize;
}

void GaSlam::cloudCallback(
            const Cloud::ConstPtr& cloud,
            const Pose& sensorToBodyTF,
            const Pose& bodyToGroundTF,
            const Pose& poseGuess) {
    const Map& map = dataRegistration_.getMap();
    std::vector<float> cloudVariances;
    const auto& sensorToMapTF = poseGuess * bodyToGroundTF * sensorToBodyTF;

    CloudProcessing::processCloud(cloud, processedCloud_, cloudVariances,
            sensorToMapTF, map, voxelSize_);

    poseEstimation_.estimatePose(map, processedCloud_,
            poseGuess * bodyToGroundTF);

    dataRegistration_.registerData(processedCloud_, cloudVariances,
            poseEstimation_.getPose());
}

}  // namespace ga_slam

