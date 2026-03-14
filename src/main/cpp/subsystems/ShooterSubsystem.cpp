// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#ifndef CHASSIS_ONLY

ShooterSubsystem::ShooterSubsystem() 
 : m_leftShooterSparkMax{ShooterConstants::kLeftShooterMotorID, ShooterConstants::kShooterMotorType},
   m_rightShooterSparkMax{ShooterConstants::kRightShooterMotorID, ShooterConstants::kShooterMotorType} {

#ifdef BURNINTAKESPARKMAX
  rev::spark::SparkMaxConfig shooterLeftSparkMaxConfig{};

  shooterLeftSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(ShooterConstants::kShooterMotorIdleMode)
  .SmartCurrentLimit(ShooterConstants::kShooterMotorCurrentLimit.value());

  m_leftShooterSparkMax.Configure(shooterLeftSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig shooterRightSparkMaxConfig{};
  shooterRightSparkMaxConfig
    .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
    .SetIdleMode(ShooterConstants::kShooterMotorIdleMode)
    .SmartCurrentLimit(ShooterConstants::kShooterMotorCurrentLimit.value())
    .Follow(ShooterConstants::kLeftShooterMotorID, true);

  m_rightShooterSparkMax.Configure(shooterRightSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Shooter");
//
//nte_coralInShooter = nt_table->GetEntry("Shooter/Fuel in Shooter");

  std::cout << "Flash Burned on shooter subsystem\r\n";
#else
  std::cout << "Flash was not burned on shooter subsystem\r\n";
#endif
}

void ShooterSubsystem::Periodic() {
//  nte_coralInShooter.SetBoolean(CoralInShooter());
}

void ShooterSubsystem::SetShooterMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_leftShooterSparkMax.Set(power);    
}

double ShooterSubsystem::GetDirection() {
  return m_leftShooterSparkMax.Get();
}

#endif