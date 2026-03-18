// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/HopperSubsystem.h"
#include "Constants.h"

#ifndef CHASSIS_ONLY

HopperSubsystem::HopperSubsystem() 
 : m_hopperSparkMax{HopperConstants::kHopperMotorID, HopperConstants::kHopperMotorType} {

#ifdef BURNINTAKESPARKMAX
  rev::spark::SparkMaxConfig hopperSparkMaxConfig{};

  hopperSparkMaxConfig
    .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
    .SetIdleMode(HopperConstants::kHopperMotorIdleMode)
    .SmartCurrentLimit(HopperConstants::kHopperMotorCurrentLimit.value());

  m_hopperSparkMax.Configure(hopperSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Intake");
//
//nte_coralInIntake = nt_table->GetEntry("Intake/Fuel in Intake");

  std::cout << "Flash Burned on intake subsystem\r\n";
#else
  std::cout << "Flash was not burned on intake subsystem\r\n";
#endif
}

void HopperSubsystem::Periodic() {
//  nte_coralInIntake.SetBoolean(CoralInIntake());
}

void HopperSubsystem::SetHopperMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_hopperSparkMax.Set(power);    
}

double HopperSubsystem::GetDirection() {
  return m_hopperSparkMax.Get();
}

#endif 