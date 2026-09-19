
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/HopperSubsystem.h"

#ifndef CHASSIS_ONLY
class MoveHopperWithTriggers
  : public frc2::CommandHelper<frc2::Command, MoveHopperWithTriggers> {
public:
  /**
   * Creates a new Simpleintake.
   *
   * @param hopper The pointer to the intake subsystem
   * 
   */
  explicit MoveHopperWithTriggers(HopperSubsystem* m_hopperSubsystem, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  HopperSubsystem* m_hopper;
  frc::XboxController* m_operatorController;
  double m_currentHopperAngle;
};
#endif