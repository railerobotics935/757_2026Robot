
#include "Constants.h"
#include "commands/shooter/ChargeShooter.h"

ChargeShooter::ChargeShooter(ShooterSubsystem *shooter, IntakeSubsystem *intake) : m_shooter{shooter}, m_intake{intake} {

  AddRequirements(m_shooter);
}

void ChargeShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Initialized\r\n";
#endif
  m_shooter->SetShooterMotorPower(-0.95);
  m_intake ->SetIntakeMotorPower(0.5);
}


void ChargeShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Ended\r\n";
#endif
  m_shooter->SetShooterMotorPower(0.0);
  m_intake->SetIntakeMotorPower(0.0);
}