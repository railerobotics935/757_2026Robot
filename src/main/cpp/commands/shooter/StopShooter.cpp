
#include "Constants.h"
#include "commands/shooter/StopShooter.h"

StopShooter::StopShooter(StagerSubsystem *stager) : m_stager{stager} {

  AddRequirements(m_stager);

}

void StopShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopShooter Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}


void StopShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}