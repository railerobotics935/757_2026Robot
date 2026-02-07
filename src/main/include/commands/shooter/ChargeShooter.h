
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"

class ChargeShooter
  : public frc2::CommandHelper<frc2::Command, ChargeShooter> {
public:
  /**
   * Creates a new SimpleShoot.
   *
   * @param stager The pointer to the stager subsystem
   * @param intake The pointer to the intake subsystem
   */
  explicit ChargeShooter(IntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
};