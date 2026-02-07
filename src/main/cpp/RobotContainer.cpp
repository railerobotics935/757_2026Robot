// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  std::cout << "RobotContainer" << std::endl;
  // Configure the button bindings
  ConfigureBindings();

  // Set Default Commands for Subsystems
  m_driveSubsystem.SetDefaultCommand(std::move(m_driveWithController));
  m_intakeSubsystem.SetDefaultCommand(std::move(m_stopIntake));
  m_stagerSubsystem.SetDefaultCommand(std::move(m_stopShooter));
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  frc2::JoystickButton intakeButton (&m_operatorController, ControllerConstants::kIntakeButton); 
  frc2::JoystickButton outakeButton (&m_operatorController, ControllerConstants::kOuttakeButton);
  frc2::JoystickButton chargeButton (&m_operatorController, ControllerConstants::kChargeButton);
  frc2::JoystickButton stageButton (&m_operatorController, ControllerConstants::kStageButton);
  frc2::JoystickButton robotRelativeButton (&m_driveController, ControllerConstants::kRobotRelativeButton);
  frc2::JoystickButton fieldRelativeButton (&m_driveController, ControllerConstants::kFieldRelativeButton);
  frc2::JoystickButton resetButton (&m_driveController, ControllerConstants::kResetButton);

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.SetFieldRelative();}, {}));
  intakeButton.WhileTrue(SimpleIntake{&m_intakeSubsystem, &m_stagerSubsystem}.ToPtr());
  outakeButton.WhileTrue(SimpleOuttake{&m_intakeSubsystem, &m_stagerSubsystem}.ToPtr());
  chargeButton.WhileTrue(ChargeShooter{&m_intakeSubsystem}.ToPtr());
  stageButton.WhileTrue(StageShooter{&m_stagerSubsystem}.ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
}
