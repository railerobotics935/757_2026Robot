
#include "Constants.h"
#include "commands/intake/SimpleOuttake.h"

SimpleOuttake::SimpleOuttake(IntakeSubsystem *intake, StagerSubsystem *stager) : m_intake{intake}, m_stager{stager} {

  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void SimpleOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(-0.7);
  m_stager->SetStagerMotorPower(-0.5);
}


void SimpleOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
  m_stager->SetStagerMotorPower(0.0);
}