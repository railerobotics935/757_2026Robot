// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"

#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SimpleOuttake.h"
#include "commands/intake/StopIntake.h"
#include "commands/shooter/SimpleShoot.h"
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

  //frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
 // frc2::CommandXboxController m_driverController{
      //OperatorConstants::kDriverControllerPort};
  
  //frc2::CommandXboxController m_operatorController{
    //OperatorConstants::kOperatorControllerPort
  //};

  // The robot's subsystems are defined here...
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  DriveSubsystem m_driveSubsystem;

  void ConfigureBindings();
  //controllers
  frc::XboxController m_driveController{OperatorConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OperatorConstants::kOperatorControllerPort};

  SimpleIntake m_simpleIntake{&m_intakeSubsystem};
  SimpleOuttake m_simpleOuttake{&m_intakeSubsystem};
  StopIntake m_stopIntake{&m_intakeSubsystem};
  SimpleShoot m_simpleShoot{&m_shooterSubsystem};
  StopShooter m_stopShooter{&m_shooterSubsystem};
  DriveWithController m_driveWithController{&m_driveSubsystem, &m_driveController};

};
