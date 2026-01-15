// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  frc2::JoystickButton intakeButton (&m_operatorController, ControllerConstants::kIntakeButton); 
  frc2::JoystickButton outakeButton (&m_operatorController, ControllerConstants::kOuttakeButton);
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  intakeButton.WhileTrue(SimpleIntake{&m_intakeSubsystem}.ToPtr());
  outakeButton.WhileTrue(SimpleOuttake{&m_intakeSubsystem}.ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

//frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
//}
