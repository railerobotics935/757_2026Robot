
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"

#ifndef CHASSIS_ONLY
class Jam
  : public frc2::CommandHelper<frc2::Command, Jam> {
public:
  /**
   * Creates a new SimpleOuttake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the stager subsystem
   */
  explicit Jam(IntakeSubsystem* intake, StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  StagerSubsystem* m_stager;
};
#endif