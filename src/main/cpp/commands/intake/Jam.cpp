#include "Constants.h"
#include "commands/intake/Jam.h"

#ifndef CHASSIS_ONLY
Jam::Jam(IntakeSubsystem *intake, StagerSubsystem *stager) : m_intake{intake}, m_stager{stager} {
  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void Jam::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(-1.0);
  m_stager->SetStagerMotorPower(1.0);
}

void Jam::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
  m_stager->SetStagerMotorPower(0.0);
}
#endif