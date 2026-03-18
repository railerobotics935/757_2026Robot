
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/HopperIntakeSubsystem.h"

#ifndef CHASSIS_ONLY
class SimpleIntake
  : public frc2::CommandHelper<frc2::Command, SimpleIntake> {
public:
  /**
   * Creates a new Simpleintake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the stager subsystem
   * @param hopperIntake The pointer to the hopper intake subsystem
   */
  explicit SimpleIntake(IntakeSubsystem* intake, StagerSubsystem* stager, HopperIntakeSubsystem* hopperIntake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;

  StagerSubsystem* m_stager;

  HopperIntakeSubsystem* m_hopperIntake;
};
#endif