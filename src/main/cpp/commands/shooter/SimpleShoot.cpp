
#include "Constants.h"
#include "commands/shooter/SimpleShoot.h"

SimpleShoot::SimpleShoot(StagerSubsystem *stager, IntakeSubsystem *intake) : m_stager{stager}, m_intake{intake} {

  AddRequirements(m_stager);
  AddRequirements(m_intake);
}

void SimpleShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(1.0);
  m_intake->SetIntakeMotorPower(-1.0);
}


void SimpleShoot::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
  m_intake->SetIntakeMotorPower(0.0);
}