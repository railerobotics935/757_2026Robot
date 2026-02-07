
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class StageShooter
  : public frc2::CommandHelper<frc2::Command, StageShooter> {
public:
  /**
   * Creates a new SimpleShoot.
   *
   * @param stager The pointer to the stager subsystem
   */
  explicit StageShooter(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};