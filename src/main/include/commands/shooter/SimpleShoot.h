
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ShooterSubsystem.h"

class SimpleShoot
  : public frc2::CommandHelper<frc2::Command, SimpleShoot> {
public:
  /**
   * Creates a new SimpleShoot.
   *
   * @param shooter The pointer to the shooter subsystem
   */
  explicit SimpleShoot(ShooterSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ShooterSubsystem* m_shooter;
};