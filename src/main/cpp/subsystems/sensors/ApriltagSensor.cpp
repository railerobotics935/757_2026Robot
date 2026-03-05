#include <vector>

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/Timer.h>

#include "subsystems/sensors/ApriltagSensor.h"

ApriltagSensor::ApriltagSensor(std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)> estConsumer)
                             : m_estConsumer{estConsumer}
{
  if (frc::RobotBase::IsSimulation()) {
    visionSim = std::make_unique<photon::VisionSystemSim>("main");

    visionSim->AddAprilTags(CameraConstants::kTagLayout);

    cameraProp = std::make_unique<photon::SimCameraProperties>();

    cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
    cameraProp->SetCalibError(.35, .10);
    cameraProp->SetFPS(15_Hz);
    cameraProp->SetAvgLatency(50_ms);
    cameraProp->SetLatencyStdDev(15_ms);

    cameraSim = std::make_shared<photon::PhotonCameraSim>(&m_camera, *cameraProp.get());

    visionSim->AddCamera(cameraSim.get(), CameraConstants::kRobotToCam);
    cameraSim->EnableDrawWireframe(true);
  }

  std::cout << "ApriltagSensor Constructor" << std::endl;

}

void ApriltagSensor::Periodic () {
//  std::cout << "ApriltagSensor Periodic" << std::endl;

  for(const auto& result : m_camera.GetAllUnreadResults()) {
    auto visionEst = m_photonEstimator.EstimateCoprocMultiTagPose(result);
    if (!visionEst) {
      //std::cout << "single tag fallback" << std::endl;
      visionEst = m_photonEstimator.EstimateLowestAmbiguityPose(result);
    }
    m_latestResult = result;

    // In sim only, add our vision estimate to the sim debug field
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst) {
        GetSimDebugField()
            .GetObject("VisionEstimation")
            ->SetPose(visionEst->estimatedPose.ToPose2d());
      } else {
        GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
      }
    }

    if (visionEst) {
      m_estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                  GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
    }
  }
}

photon::PhotonPipelineResult ApriltagSensor::GetLatestResult() { 
  return m_latestResult; 
}

Eigen::Matrix<double, 3, 1> ApriltagSensor::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
  Eigen::Matrix<double, 3, 1> estStdDevs = CameraConstants::kSingleTagStdDevs;

  auto targets = m_latestResult.GetTargets();
  int numTags = 0;
  units::meter_t avgDist = 0_m;
  for (const auto& tgt : targets) {
    auto tagPose = m_photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose) {
      numTags++;
      avgDist += tagPose->ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    return estStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = CameraConstants::kMultiTagStdDevs;
  }
  if (numTags == 1 && avgDist > 4_m) {
    estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max())
                      .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
  }
  std::cout << estStdDevs << std::endl;
  return estStdDevs;
}

void ApriltagSensor::SimulationPeriodic(frc::Pose2d robotSimPose) {
  visionSim->Update(robotSimPose);
}

void ApriltagSensor::ResetSimPose(frc::Pose2d pose) {
  if (frc::RobotBase::IsSimulation()) {
    visionSim->ResetRobotPose(pose);
  }
}

frc::Field2d& ApriltagSensor::GetSimDebugField() { 
  return visionSim->GetDebugField(); 
}
