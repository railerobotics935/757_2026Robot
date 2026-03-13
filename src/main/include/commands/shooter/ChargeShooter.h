
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#ifndef CHASSIS_ONLY
class ChargeShooter
  : public frc2::CommandHelper<frc2::Command, ChargeShooter> {
public:
  /**
   * Creates a new SimpleShoot.
   *
   * @param shooter The pointer to the shooter subsystem
   */
  explicit ChargeShooter(ShooterSubsystem* shooter,IntakeSubsystem* intake);
  
  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ShooterSubsystem* m_shooter;
  IntakeSubsystem* m_intake;
};
#endif