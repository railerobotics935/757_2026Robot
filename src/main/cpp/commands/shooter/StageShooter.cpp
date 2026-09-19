#include "Constants.h"
#include "commands/shooter/StageShooter.h"

#ifndef CHASSIS_ONLY
StageShooter::StageShooter(StagerSubsystem *stager, IntakeSubsystem *intake) : m_stager{stager}, m_intake{intake} {
  AddRequirements(m_stager);
  AddRequirements(m_intake);

}

void StageShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StageShooter Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(-1.0);
  m_intake->SetIntakeMotorPower(1.0);
}

void StageShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StageShooter Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
  m_intake->SetIntakeMotorPower(0.0);
}
#endif