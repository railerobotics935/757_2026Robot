// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem() 

: m_intakeSparkMax{IntakeConstants::kIntakeTopMotorID, IntakeConstants::kIntakeMotorType} {


   #ifdef BURNINTAKESPARKMAX

  rev::spark::SparkMaxConfig intakeSparkMaxConfig{};

  intakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  m_intakeSparkMax.Configure(intakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Intake");
//
//nte_coralInIntake = nt_table->GetEntry("Intake/Fuel in Intake");

  std::cout << "Flash Burned on intake subsystem\r\n";
  #else
  std::cout << "Flash was not burned on intake subsystem\r\n";
  #endif
}

/*
void IntakeSubsystem::Periodic() {
  nte_coralInIntake.SetBoolean(CoralInIntake());
}
*/
void IntakeSubsystem::SetIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeSparkMax.Set(power);    

}

double IntakeSubsystem::GetDirection() {
  return m_intakeSparkMax.Get();
}
