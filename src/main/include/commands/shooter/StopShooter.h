
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class StopShooter
  : public frc2::CommandHelper<frc2::Command, StopShooter> {
public:
  /**
   * Creates a new StopShooter.
   *
   * @param stager The pointer to the stager subsystem
   */
  explicit StopShooter(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};