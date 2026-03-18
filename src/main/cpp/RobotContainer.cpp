// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>


#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

using namespace pathplanner;


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  std::cout << "RobotContainer" << std::endl;
  // Configure the button bindings
  ConfigureBindings();

  // Set Default Commands for Subsystems
  m_driveSubsystem.SetDefaultCommand(std::move(m_driveWithController));
#ifndef CHASSIS_ONLY
  m_intakeSubsystem.SetDefaultCommand(std::move(m_stopIntake));
  m_stagerSubsystem.SetDefaultCommand(std::move(m_stopStager));
  m_shooterSubsystem.SetDefaultCommand(std::move(m_stopShooter));
#endif

NamedCommands::registerCommand("Fuel Intake", std::move(m_simpleIntake).ToPtr());
NamedCommands::registerCommand("Stop Fuel Intake", std::move(m_stopIntake).ToPtr());
NamedCommands::registerCommand("Stop Shooter", std::move(m_stopShooter).ToPtr());
NamedCommands::registerCommand("Stop Stager", std::move(m_stopStager).ToPtr());
NamedCommands::registerCommand("Charge Shooter", std::move(m_chargeShooter).ToPtr());
NamedCommands::registerCommand("Shoot", std::move(m_stageShooter).ToPtr());
  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);
  m_autoChooser.SetDefaultOption("Basic Auto C", m_defaultAuto);
  m_autoChooser.AddOption("Basic Auto L", m_basicAutoL);
  m_autoChooser.AddOption("Basic Auto R", m_basicAutoR);
  m_autoChooser.AddOption("BlueRightTrench", m_blueRightTrench);
  m_autoChooser.AddOption("BlueRightBump", m_blueRightBump);
}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  frc2::JoystickButton intakeButton (&m_operatorController, ControllerConstants::kIntakeButton); 
  frc2::JoystickButton outakeButton (&m_operatorController, ControllerConstants::kOuttakeButton);
  frc2::JoystickButton chargeButton (&m_operatorController, ControllerConstants::kChargeButton);
  frc2::JoystickButton extendHopperButton (&m_operatorController, ControllerConstants::kExtendHopperButton);
  frc2::JoystickButton retractHopperButton (&m_operatorController, ControllerConstants::kRetractHopperButton);
  frc2::JoystickButton stageButton (&m_operatorController, ControllerConstants::kStageButton);
  frc2::JoystickButton robotRelativeButton (&m_driveController, ControllerConstants::kRobotRelativeButton);
  frc2::JoystickButton fieldRelativeButton (&m_driveController, ControllerConstants::kFieldRelativeButton);
  frc2::JoystickButton resetButton (&m_driveController, ControllerConstants::kResetButton);


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.SetFieldRelative();}, {}));
#ifndef CHASSIS_ONLY
  intakeButton.WhileTrue(SimpleIntake{&m_intakeSubsystem, &m_stagerSubsystem, &m_hopperIntakeSubsystem}.ToPtr());
  outakeButton.WhileTrue(SimpleOuttake{&m_intakeSubsystem, &m_stagerSubsystem}.ToPtr());
  chargeButton.WhileTrue(ChargeShooter{&m_shooterSubsystem, &m_intakeSubsystem}.ToPtr());
  stageButton.WhileTrue(StageShooter{&m_stagerSubsystem}.ToPtr());
  extendHopperButton.WhileTrue(ExtendHopper{&m_hopperSubsystem}.ToPtr());
  retractHopperButton.WhileTrue(RetractHopper{&m_hopperSubsystem}.ToPtr());
#endif
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return pathplanner::PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
