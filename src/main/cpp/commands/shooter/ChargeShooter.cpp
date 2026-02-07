
#include "Constants.h"
#include "commands/shooter/ChargeShooter.h"

ChargeShooter::ChargeShooter(IntakeSubsystem *intake) : m_intake{intake} {

  AddRequirements(m_intake);
}

void ChargeShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.85);
}


void ChargeShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}