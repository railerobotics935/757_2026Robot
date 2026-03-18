// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/HopperIntakeSubsystem.h"
#include "Constants.h"

#ifndef CHASSIS_ONLY

HopperIntakeSubsystem::HopperIntakeSubsystem() 
 : m_hopperIntakeSparkMax{HopperIntakeConstants::kHopperIntakeMotorID, HopperIntakeConstants::kHopperIntakeMotorType} {

#ifdef BURNINTAKESPARKMAX
  rev::spark::SparkMaxConfig hopperIntakeSparkMaxConfig{};

  hopperIntakeSparkMaxConfig
    .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
    .SetIdleMode(IntakeConstants::kHopperIntakeMotorIdleMode)
    .SmartCurrentLimit(IntakeConstants::kHopperIntakeMotorCurrentLimit.value());

  m_hopperIntakeSparkMax.Configure(hopperIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Intake");
//
//nte_coralInIntake = nt_table->GetEntry("Intake/Fuel in Intake");

  std::cout << "Flash Burned on intake subsystem\r\n";
#else
  std::cout << "Flash was not burned on intake subsystem\r\n";
#endif
}

void HopperIntakeSubsystem::Periodic() {
//  nte_coralInIntake.SetBoolean(CoralInIntake());
}

void HopperIntakeSubsystem::SetHopperIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_hopperIntakeSparkMax.Set(power);    
}

double HopperIntakeSubsystem::GetDirection() {
  return m_hopperIntakeSparkMax.Get();
}

#endif 