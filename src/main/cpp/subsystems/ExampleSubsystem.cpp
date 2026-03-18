// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"
#include "frc/Timer.h"
#include "units/time.h"

#ifndef CHASSIS_ONLY

ExampleSubsystem::ExampleSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ExampleSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ExampleSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  //Get time at start
  units::second_t timestamp_seconds_1 = frc::Timer::GetFPGATimestamp();


  //ROBOT PERIODITC THINGS HAPPEN

  // Get time at end
  units::second_t timestamp_seconds_2 = frc::Timer::GetFPGATimestamp();

  // Take differnce


  // if end - start is > 0.05, print subsystem name and time
}

void ExampleSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

#endif