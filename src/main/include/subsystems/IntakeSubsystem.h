// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <iostream>
#include <Constants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  IntakeSubsystem();

  // Sets the motor's power (between -1.0 and 1.0).
  
  void Periodic() override;

  
  void SetIntakeMotorPower(double power);

  /**
   * @return Direction intake motor is moving
   */
  double GetDirection();

  /**
   * @return If light sensor has detected a coral
   */
  //bool CoralInIntake();

 private:

  void ConfigureSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_intakeSparkMax;

  // Light Sensor is a digital input in the DIO port (digital input output)
  //frc::DigitalInput m_lightSensor{IntakeConstants::kLightSensorID};


  //Network Table Entry
  //nt::NetworkTableEntry nte_coralInIntake;
};
