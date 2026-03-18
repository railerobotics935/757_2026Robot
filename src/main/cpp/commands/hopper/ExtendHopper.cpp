
#include "Constants.h"
#include "commands/hopper/ExtendHopper.h"

#ifndef CHASSIS_ONLY
ExtendHopper::ExtendHopper(HopperSubsystem *hopper) : m_hopper{hopper} {
  AddRequirements(m_hopper);
}

void ExtendHopper::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_hopper->SetHopperMotorPower(-0.5);

}


void ExtendHopper::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_hopper->SetHopperMotorPower(0.0);
  }
#endif