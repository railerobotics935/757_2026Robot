
#include "Constants.h"
#include "commands/shooter/StopShooter.h"

StopShooter::StopShooter(StagerSubsystem *shooter) : m_shooter{shooter} {

  AddRequirements(m_shooter);
}

void StopShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_shooter->SetStagerMotorPower(0.0);
}


void StopShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_shooter->SetStagerMotorPower(0.0);
}