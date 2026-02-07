
#include "Constants.h"
#include "commands/shooter/StageShooter.h"

StageShooter::StageShooter(StagerSubsystem *stager) : m_stager{stager} {

  AddRequirements(m_stager);
}

void StageShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StageShooter Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(-0.25);
}


void StageShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StageShooter Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}