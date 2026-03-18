
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/HopperSubsystem.h"

#ifndef CHASSIS_ONLY
class ExtendHopper
  : public frc2::CommandHelper<frc2::Command, ExtendHopper> {
public:
  /**
   * Creates a new Simpleintake.
   *
   * @param hopper The pointer to the intake subsystem
   * 
   */
  explicit ExtendHopper(HopperSubsystem* m_hopperSubsystem);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  HopperSubsystem* m_hopper;
};
#endif