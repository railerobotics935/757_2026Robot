
#include "Constants.h"
#include "commands/intake/SimpleIntake.h"

#ifndef CHASSIS_ONLY
SimpleIntake::SimpleIntake(IntakeSubsystem *intake, StagerSubsystem *stager, HopperIntakeSubsystem *hopperIntake) : m_intake{intake}, m_stager{stager}, m_hopperIntake{hopperIntake} {
  AddRequirements(m_intake);
  AddRequirements(m_stager);
  AddRequirements(m_hopperIntake);
}

void SimpleIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.95);
  m_stager->SetStagerMotorPower(1.0);
  m_hopperIntake->SetHopperIntakeMotorPower(0.95);
}


void SimpleIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
  m_stager->SetStagerMotorPower(0.0);
  m_hopperIntake->SetHopperIntakeMotorPower(0.0);
}
#endif