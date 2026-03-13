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

#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SimpleOuttake.h"
#include "commands/intake/StopIntake.h"
#include "commands/shooter/ChargeShooter.h"
#include "commands/shooter/StageShooter.h"
#include "commands/shooter/StopStager.h"
#include "commands/shooter/StopShooter.h"
#include "commands/drive/DriveWithController.h"


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
#endif
  DriveSubsystem m_driveSubsystem;
  

  ApriltagSensor m_apriltagSensor{[=, this](frc::Pose2d pose, units::second_t timestamp) {
    m_driveSubsystem.AddVisionMeasurement(pose, timestamp);
  }};

  void ConfigureBindings();
  
  // Controllers
  frc::XboxController m_driveController{OperatorConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OperatorConstants::kOperatorControllerPort};

  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  // Autos
  std::string m_testAuto = "Test Auto";

  // Commands
 #ifndef CHASSIS_ONLY
  SimpleIntake m_simpleIntake{&m_intakeSubsystem, &m_stagerSubsystem};
  SimpleOuttake m_simpleOuttake{&m_intakeSubsystem, &m_stagerSubsystem};
  StopIntake m_stopIntake{&m_intakeSubsystem};
  ChargeShooter m_chargeShooter{&m_shooterSubsystem, &m_intakeSubsystem};
  StageShooter m_stageShooter{&m_stagerSubsystem};
  StopShooter m_stopShooter{&m_shooterSubsystem};
  StopStager m_stopStager{&m_stagerSubsystem};
#endif
  DriveWithController m_driveWithController{&m_driveSubsystem, &m_driveController};
};
