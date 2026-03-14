#pragma once

#include <networktables/NetworktableEntry.h>
#include <networktables/networkTableInstance.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <wpi/fs.h>
#include <wpi/array.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/geometry/Pose2d.h>

#include <string.h>

#include "Constants.h"

//#define MAX_NUM_TAGS 16
#ifdef CAMERAS 
class ApriltagSensor : public frc2::SubsystemBase{
public:
    /**
   * ApriltagSensor is meant to be implemnted as any other sensor for the robot
   * but simply takes information from Network tables and oragnizes it for use 
   * in the robot code
   * 
   * @param cameraName The camera name from network tables
   * @param cameraPose3d The 3d position of the camera 
  */
  ApriltagSensor (std::function<void(frc::Pose2d, units::second_t)> estConsumer);

  photon::PhotonPipelineResult GetLatestResult();

  void Periodic() override;

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

  virtual void SimulationPeriodic(frc::Pose2d robotSimPose);
  void ResetSimPose(frc::Pose2d pose);
  frc::Field2d& GetSimDebugField();


private:

  photon::PhotonPoseEstimator m_photonEstimator{CameraConstants::kTagLayout,
                                              CameraConstants::kRobotToCam};
  photon::PhotonPoseEstimator m_photonEstimator2{CameraConstants::kTagLayout,
                                              CameraConstants::kRobotToCam2};
  photon::PhotonCamera m_camera{CameraConstants::kCameraName};
  photon::PhotonCamera m_camera2{CameraConstants::kCamera2Name};
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;

  photon::PhotonPipelineResult m_latestResult;
  photon::PhotonPipelineResult m_latestResult2;
  std::function<void(frc::Pose2d, units::second_t)> m_estConsumer;
};
#endif //CAMERAS