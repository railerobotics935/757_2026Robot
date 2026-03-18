
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/HopperSubsystem.h"

#ifndef CHASSIS_ONLY
class RetractHopper
  : public frc2::CommandHelper<frc2::Command, RetractHopper> {
public:
  /**
   * Creates a new Simpleintake.
   *
   * @param hopper The pointer to the intake subsystem
   * 
   */
  explicit RetractHopper(HopperSubsystem* m_hopperSubsystem);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  HopperSubsystem* m_hopper;
};
#endif