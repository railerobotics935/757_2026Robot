
#include "Constants.h"
#include "commands/hopper/RetractHopper.h"

#ifndef CHASSIS_ONLY
RetractHopper::RetractHopper(HopperSubsystem *hopper) : m_hopper{hopper} {
  AddRequirements(m_hopper);
}

void RetractHopper::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
//  m_hopper->SetHopperMotorPower(0.3);
  m_currentArmAngle = m_hopper->GetAngle();
}

void RetractHopper::Execute() {
  m_currentArmAngle -= 0.01;

  m_hopper->SetAngle(m_currentArmAngle);
}

void RetractHopper::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_hopper->SetHopperMotorPower(0.0);
  }
#endif