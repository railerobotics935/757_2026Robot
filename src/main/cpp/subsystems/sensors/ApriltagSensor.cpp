#include <vector>

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/Timer.h>


#include "subsystems/sensors/ApriltagSensor.h"


ApriltagSensor::ApriltagSensor(std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)> estConsumer) : estConsumer{estConsumer} 
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
  #if 0
	// Set the camera name to identify whitch camera to look at in NT
	m_cameraName = cameraName;
  m_cameraPose3d = cameraPose3d;
  m_cameraTransform2d = {m_cameraPose3d.ToPose2d().Translation(), m_cameraPose3d.ToPose2d().Rotation()};
  #endif

}

void ApriltagSensor::Periodic () {

  for(const auto& result : m_camera.GetAllUnreadResults()) {
    auto visionEst = m_photonEstimator.EstimateCoprocMultiTagPose(result);
    m_latestResult = result;
      if (!visionEst) {
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
    return estStdDevs;
}

void ApriltagSensor::SimulationPeriodic(frc::Pose2d robotSimPose) {
  visionSim->Update(robotSimPose);
}

void ApriltagSensor::ResetSimPose(frc::Pose2d pose) {
  
}

frc::Field2d& ApriltagSensor::GetSimDebugField() { 
  return visionSim->GetDebugField(); 
}

#if 0
frc::Pose3d ApriltagSensor::GetFieldRelativePose(int tag) {
  // Grab Pose3d values in an vector
  m_poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());

  // Create Transform3d object for tag position relative to robot
  m_rawTranslation = {(units::meter_t)m_poseArr[0], (units::meter_t)m_poseArr[1], (units::meter_t)m_poseArr[2]};
  m_rawRotation = {(units::radian_t)m_poseArr[3], (units::radian_t)m_poseArr[4], (units::radian_t)m_poseArr[5]};
  m_rawPose = {m_rawTranslation, m_rawRotation};
  
  // Correct rotation so the robot is rotated, not the apriltag - correct axis manualy and add pi to the rotation axis to face tag instead
  m_correctedRotation = {(units::radian_t)m_poseArr[5], (units::radian_t)m_poseArr[3], (units::radian_t)(m_poseArr[4] + std::numbers::pi)}; 

  // Convert translation into standard for the robot
  m_convertedTranslation = frc::CoordinateSystem::Convert(m_rawPose.Translation().RotateBy(m_rawPose.Inverse().Rotation()), 
                                frc::CoordinateSystem::EDN(), 
                                frc::CoordinateSystem::NWU());
  
  // Final Transformation
  m_calculatedRobotPose = frc::Pose3d{m_convertedTranslation, m_correctedRotation}.operator+(frc::Transform3d{-(m_cameraPose3d.Translation().RotateBy(-m_cameraPose3d.Rotation())), -m_cameraPose3d.Rotation()});

  return m_fieldLayout.GetTagPose(tag).value().TransformBy(frc::Transform3d{m_calculatedRobotPose.Translation(), m_calculatedRobotPose.Rotation()});
}

//wpi::array<double, 3> ApriltagSensor::GetStandardDeviations(int tag) {
  // Grab Pose3d values in an vector
 // m_poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // use the raw distance to get the information
  //double standardDeviation = CameraConstants::GetStandardDeviationFromDistance((double)m_poseArr[2]);

  //return wpi::array<double, 3>{standardDeviation, standardDeviation, standardDeviation / 20.0};
//}

bool ApriltagSensor::TagIsTracked(int tag) {
  // If tag is traked, return true, else return false
  if (nte_status[tag].GetString("LOST") == "TRACKED")
    return true;
  else
    return false;
}

units::second_t ApriltagSensor::GetTimestamp(int tag) {
  units::second_t timestamp = (units::second_t)nte_latency.GetDouble(360.0) + (units::second_t)(nte_pose[tag].GetLastChange() / 1000000.0);
  nte_finalLatency.SetDouble((double)timestamp);
  return (units::second_t)timestamp;
}

bool ApriltagSensor::HasNewData(int tag) {
  return true;
  
  if (nte_pose[tag].GetLastChange() - m_prevLatency != 0) {
    m_prevLatency = nte_pose[tag].GetLastChange();
    return true;
  }
  else
    return false;
}
#endif