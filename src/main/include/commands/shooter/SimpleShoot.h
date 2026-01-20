
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class SimpleShoot
  : public frc2::CommandHelper<frc2::Command, SimpleShoot> {
public:
  /**
   * Creates a new SimpleShoot.
   *
   * @param shooter The pointer to the shooter subsystem
   */
  explicit SimpleShoot(StagerSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_shooter;
};