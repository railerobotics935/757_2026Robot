// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

ShooterSubsystem::ShooterSubsystem() 

: m_rightShooterSparkMax{ShooterConstants::kShooterRightMotorID, ShooterConstants::kShooterMotorType},
  m_leftShooterSparkMax{ShooterConstants::kShooterLeftMotorID, ShooterConstants::kShooterMotorType} {


   #ifdef BURNSHOOTERSPARKMAX

  rev::spark::SparkMaxConfig rightShooterSparkMaxConfig{};

  rightShooterSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(ShooterConstants::kShooterMotorIdleMode)
  .SmartCurrentLimit(ShooterConstants::kShooterMotorCurrentLimit.value());

  m_rightShooterSparkMax.Configure(rightShooterSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig leftShooterSparkMaxConfig{};

  leftShooterSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(ShooterConstants::kShooterMotorIdleMode)
  .SmartCurrentLimit(ShooterConstants::kShooterMotorCurrentLimit.value())
  .Follow(ShooterConstants::kShooterRightMotorID, true);

   m_leftShooterSparkMax.Configure(leftShooterSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);


//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Shooter");
//
//nte_coralInShooter = nt_table->GetEntry("Shooter/Fuel in Shooter");

  std::cout << "Flash Burned on shooter subsystem\r\n";
  #else
  std::cout << "Flash was not burned on shooter subsystem\r\n";
  #endif
}

/*
void ShooterSubsystem::Periodic() {
  nte_coralInShooter.SetBoolean(CoralInShooter());
}
*/

void ShooterSubsystem::SetShooterMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_rightShooterSparkMax.Set(power);    

}

double ShooterSubsystem::GetDirection() {
  return m_rightShooterSparkMax.Get();
}
