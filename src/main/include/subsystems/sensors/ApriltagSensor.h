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


#define MAX_NUM_TAGS 16


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
  ApriltagSensor (std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)> estConsumer);


  photon::PhotonPipelineResult GetLatestResult();

  void Periodic() override;

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

  virtual void SimulationPeriodic(frc::Pose2d robotSimPose);
  void ResetSimPose(frc::Pose2d pose);
  frc::Field2d& GetSimDebugField();

#if 0
  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Field Relative Pose3d
  */
  frc::Pose3d GetFieldRelativePose(int tag);

  /**
   * @param tag the ID number for the apriltag wanted to identify
   * @return An array of standard deviations scaled by the distance form the shooter
  */
  wpi::array<double, 3> GetStandardDeviations(int tag);

  /**
   * @return If the tag is tracked
  */
  bool TagIsTracked(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Timestamp of a pose
  */
  units::second_t GetTimestamp(int tag);

  /**
   * Uses the timestamp information to deterime if the camera has new data compared to the last time ran
   * 
   * @return True if the camera has new data to process
  */
 bool HasNewData(int tag);
#endif

private:
  // Declare Network table entrys for apriltag pos
  nt::NetworkTableEntry nte_status[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_pose[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_latency;
  nt::NetworkTableEntry nte_finalLatency;
  nt::NetworkTableEntry nte_estimatedRobotPose;

  std::string m_cameraName;
  frc::Pose3d m_cameraPose3d;
  frc::Transform2d m_cameraTransform2d;
  frc::Pose3d m_calculatedRobotPose;
  
  // Create path to deploy directory
  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/2026-rebuilt.json"};

  // Field layout to get apriltag pose
  frc::AprilTagFieldLayout m_fieldLayout{deployDirectory.string()};
  
  frc::Timer m_timer{};
  int64_t m_prevLatency = 0;

  // Local variables resued in processing
  std::vector<double> m_poseArr{};

  frc::Translation3d m_rawTranslation{};
  frc::Rotation3d m_rawRotation{};
  frc::Transform3d m_rawPose{};

  frc::Translation3d m_convertedTranslation{};
  frc::Rotation3d m_correctedRotation{};

  
  photon::PhotonPoseEstimator m_photonEstimator{CameraConstants::kTagLayout,
                                              CameraConstants::kRobotToCam};

  photon::PhotonCamera m_camera{CameraConstants::kCameraName};

  photon::PhotonPipelineResult m_latestResult;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;

};