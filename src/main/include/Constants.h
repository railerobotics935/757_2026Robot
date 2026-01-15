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

 #define BURNINTAKESPARKMAX

namespace RobotConstants {

constexpr double kVoltageCompentationValue = 11.0;

const units::meter_t kWheelBase =
    0.6731_m; 
const units::meter_t kWheelWidth =
    0.4953_m; 


}  // namespace RobotConstants

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;

}  // namespace OperatorConstants

namespace ControllerConstants {

constexpr int kIntakeButton = 3;
constexpr int kOuttakeButton =4;

} //ControllerConstants

namespace IntakeConstants {

// Intake Motors
constexpr int kIntakeTopMotorID = 18;
constexpr int kIntakeBottomMotorID = 19;

constexpr rev::spark::SparkLowLevel::MotorType kIntakeMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kIntakeMotorCurrentLimit = 40_A;

}  // namespace IntakeConstants
