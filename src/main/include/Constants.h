// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <rev/SparkMax.h>
#include <iostream>
#include <rev/config/SparkMaxConfig.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

//#define BURNINTAKESPARKMAX
//#define BURNSHOOTERSPARKMAX
//#define BURNMODULESPARKMAX
//#define PRINTDEBUG
//#define SETTURNINGZEROS
//#define CHASSIS_ONLY
//#define CAMERAS
//#define PID_TUNING_FROM_ELASTIC

namespace RobotConstants {

constexpr double kVoltageCompentationValue = 11.0;

const units::meter_t kWheelBase =
     0.42545_m;
const units::meter_t kWheelWidth =
     0.70485_m;
}  // namespace RobotConstants

namespace ModuleConstants {
// Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0953_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;

// 6.75:1 Gear Ratio for Driving Motors
constexpr double kDrivingMotorReduction = 6.75;
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.0;
constexpr double kDrivingI = 0.0;
constexpr double kDrivingD = 0.0;
constexpr double kDrivingFF = (4 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

//constexpr double kTurningP = 5.5; //0.6*Kc
constexpr double kTurningP = 1.0; //0.6*Kc 0.5
constexpr double kTurningI = 0.0;  //Ki = 2*KP/Pc
constexpr double kTurningD = 0.0; //was originally 0.004 0.125*Kp/Pc
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr double kFrontLeftEncoderOffset = 0;//(2.79 / (std::numbers::pi * 2));
constexpr double kFrontRightEncoderOffset = 0;//(3.58 / (std::numbers::pi * 2));
constexpr double kBackLeftEncoderOffset = 0;//(1.71 / (std::numbers::pi * 2));
constexpr double kBackRightEncoderOffset = 0;//(2.74 / (std::numbers::pi * 2));

constexpr rev::spark::SparkMaxConfig::IdleMode kDrivingMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurningMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 40_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.65_mps;
}  // namespace ModuleConstants

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2.0 * std::numbers::pi};
constexpr double kDirectionSlewRate = 6.0;   // radians per second
constexpr double kMagnitudeSlewRate = 7.0;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 8.0;  // percent per second (1 = 100%)

// CAN Sparkmax id numbers
constexpr int kFrontLeftTurningMotorPort = 21;
constexpr int kFrontRightTurningMotorPort = 13;
constexpr int kBackLeftTurningMotorPort = 19;
constexpr int kBackRightTurningMotorPort = 15;

constexpr int kFrontLeftDriveMotorPort = 24;
constexpr int kFrontRightDriveMotorPort = 12;
constexpr int kBackLeftDriveMotorPort = 22;
constexpr int kBackRightDriveMotorPort = 14;

//Analog id numbers
constexpr int kFrontLeftAnalogId = 4;
constexpr int kFrontRightAnalogId = 1;
constexpr int kBackLeftAnalogId = 2;
constexpr int kBackRightAnalogId = 3;

// PID Controller for the auto rotation of the robot
constexpr double kRotationP = 2;
constexpr double kRotationI = 0.0;
constexpr double kRotationD = 0.05;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi

constexpr auto kDriveBaseRadius = 0.4248_m;

}  // namespace DriveConstants

namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 4.0;//4.0; // 6.0
constexpr double kITanslationController = 1.7; // 1.7
constexpr double kDTanslationController = 0.0; // 0.0

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 5.0; // 5.0
constexpr double kIRotationController = 0.0; // 0.0
constexpr double kDRotationController = 0.0; // 0.0

}  // namespace AutoConstants

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;

}  // namespace OperatorConstants

namespace ControllerConstants {

//Driver Controls
//Xbox Controller
//Driver Axis
constexpr int kDriveLeftXIndex = 0;
constexpr int kDriveLeftYIndex = 1;
constexpr int kDriveRightXIndex = 4;

//Driver Buttons
constexpr int kFieldRelativeButton = 7;
constexpr int kRobotRelativeButton = 8;
constexpr int kResetButton = 2;

//Operator Controls
//Xbox Controller
//Operator Buttons
constexpr int kIntakeButton = 6; //RT
constexpr int kOuttakeButton = 2; //B
constexpr int kChargeButton = 5; //LT
constexpr int kStageButton = 3; //X
constexpr int kExtendHopperButton = 4; // Y
constexpr int kRetractHopperButton = 1; //A
} //ControllerConstants

namespace IntakeConstants {

// Intake Motors
constexpr int kIntakeMotorID = 20; 

constexpr rev::spark::SparkLowLevel::MotorType kIntakeMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kIntakeMotorCurrentLimit = 40_A;

}  // namespace IntakeConstants

namespace ShooterConstants {

// Shooter Motors
constexpr int kLeftShooterMotorID = 18;
constexpr int kRightShooterMotorID = 17;

constexpr rev::spark::SparkLowLevel::MotorType kShooterMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kShooterMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kShooterMotorCurrentLimit = 40_A;
} // namespace ShooterConstants

namespace StagerConstants {

// Shooter Motors
constexpr int kStagerMotorID = 16;

constexpr rev::spark::SparkLowLevel::MotorType kStagerMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kStagerMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kStagerMotorCurrentLimit = 40_A;

}  // namespace StagerConstants

namespace HopperConstants{
// Shooter Motors
constexpr int kHopperMotorID = 29;

constexpr rev::spark::SparkLowLevel::MotorType kHopperMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kHopperMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kHopperMotorCurrentLimit = 40_A;
}

namespace HopperIntakeConstants{
// Shooter Motors
constexpr int kHopperIntakeMotorID = 10;

constexpr rev::spark::SparkLowLevel::MotorType kHopperIntakeMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kHopperIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kHopperIntakeMotorCurrentLimit = 40_A;
}


namespace CameraConstants {

constexpr double kYawP = 0.05;
constexpr double kYawI = 0.0;
constexpr double kYawD = 0.01;

constexpr double kPitchP = 0.3;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;

constexpr int counterMax = 250;
constexpr int yawCounterMax = 50;

// Min and Max standard deviations for the apriltag detetion 
constexpr double kMinStandardDeviation = 0.2;
constexpr double kMaxStandardDeviation = 3.0;

// Max speed allowed for adding vidion measurments to the robot pose esitmator
constexpr double kMaxEstimationSpeed = 0.25; // mps

inline constexpr std::string_view kCameraName{"Camera1"};
inline constexpr std::string_view kCamera2Name{"Camera2"};
inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{0.313_m, 0.289_m, 0.164_m},
    frc::Rotation3d{0_rad, 0_rad, 0_deg}};

inline const frc::Transform3d kRobotToCam2{
    frc::Translation3d{-0.31_m, 0.34_m, 0.1_m},
    frc::Rotation3d{14_deg, 0_deg, 180_deg}};
//    frc::Rotation3d{0_rad, -20_deg, 0_rad}};
inline const frc::AprilTagFieldLayout kTagLayout{
   frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark)}; 

inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};


/**
 * @param distance The raw distance from the apriltag
 * 
 * @return The standard deviation value for the distance
*/
double GetStandardDeviationFromDistance(double distance);

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.250, (units::meter_t)0.0, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)std::numbers::pi / 12, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
    
} // namespace FrontCamera

namespace OakDLiteCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.08, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)std::numbers::pi};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace OakDLiteCamera

} // namespace CameraConstants
