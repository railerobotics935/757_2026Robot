
#include "Constants.h"
#include "commands/shooter/SimpleShoot.h"

SimpleShoot::SimpleShoot(StagerSubsystem *shooter) : m_shooter{shooter} {

  AddRequirements(m_shooter);
}

void SimpleShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_shooter->SetStagerMotorPower(1.0);
}


void SimpleShoot::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_shooter->SetStagerMotorPower(0.0);
}