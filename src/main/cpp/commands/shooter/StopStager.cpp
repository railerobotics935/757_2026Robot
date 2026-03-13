
#include "Constants.h"
#include "commands/shooter/StopStager.h"

#ifndef CHASSIS_ONLY
StopStager::StopStager(StagerSubsystem *stager) : m_stager{stager} {

  AddRequirements(m_stager);

}

void StopStager::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopShooter Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}


void StopStager::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}
#endif