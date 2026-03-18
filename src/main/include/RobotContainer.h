// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "pathplanner/lib/commands/PathPlannerAuto.h"

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/sensors/ApriltagSensor.h"
#include "subsystems/HopperIntakeSubsystem.h"
#include "subsystems/HopperSubsystem.h"

#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SimpleOuttake.h"
#include "commands/intake/StopIntake.h"
#include "commands/shooter/ChargeShooter.h"
#include "commands/shooter/StageShooter.h"
#include "commands/shooter/StopStager.h"
#include "commands/shooter/StopShooter.h"
#include "commands/drive/DriveWithController.h"
#include "commands/hopper/ExtendHopper.h"
#include "commands/hopper/RetractHopper.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
 // frc2::CommandXboxController m_driverController{
      //OperatorConstants::kDriverControllerPort};
  
  //frc2::CommandXboxController m_operatorController{
    //OperatorConstants::kOperatorControllerPort
  //};

  // The robot's subsystems are defined here
#ifndef CHASSIS_ONLY
  IntakeSubsystem m_intakeSubsystem;
  StagerSubsystem m_stagerSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  HopperIntakeSubsystem m_hopperIntakeSubsystem;
  HopperSubsystem m_hopperSubsystem;
#endif
  DriveSubsystem m_driveSubsystem;
  
#ifdef CAMERAS
  ApriltagSensor m_apriltagSensor{[=, this](frc::Pose2d pose, units::second_t timestamp) {
    m_driveSubsystem.AddVisionMeasurement(pose, timestamp);
  }};

  #endif //CAEMEAS

  void ConfigureBindings();
  
  // Controllers
  frc::XboxController m_driveController{OperatorConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OperatorConstants::kOperatorControllerPort};

  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  // Autos
  std::string m_defaultAuto = "Basic Auto C";
  std::string m_basicAutoL = "Basic Auto L";
  std::string m_basicAutoR = "Basic Auto R";
  std::string m_blueRightTrench = "BlueRightTrench";
  std::string m_blueRightBump = "BlueRightBump";

  // Commands
 #ifndef CHASSIS_ONLY
  SimpleIntake m_simpleIntake{&m_intakeSubsystem, &m_stagerSubsystem, &m_hopperIntakeSubsystem};
  SimpleOuttake m_simpleOuttake{&m_intakeSubsystem, &m_stagerSubsystem};
  StopIntake m_stopIntake{&m_intakeSubsystem};
  ChargeShooter m_chargeShooter{&m_shooterSubsystem, &m_intakeSubsystem};
  StageShooter m_stageShooter{&m_stagerSubsystem};
  StopShooter m_stopShooter{&m_shooterSubsystem};
  StopStager m_stopStager{&m_stagerSubsystem};
  ExtendHopper m_extendHopper{&m_hopperSubsystem};
  RetractHopper m_retractHopper{&m_hopperSubsystem};
#endif
  DriveWithController m_driveWithController{&m_driveSubsystem, &m_driveController};
};
