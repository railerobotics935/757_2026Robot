
#include "Constants.h"
#include "commands/hopper/ExtendHopper.h"

#ifndef CHASSIS_ONLY
ExtendHopper::ExtendHopper(HopperSubsystem *hopper, frc::XboxController *operatorController) : m_hopper{hopper}, m_operatorController{operatorController} {
  AddRequirements(m_hopper);
}

void ExtendHopper::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_currentHopperAngle = m_hopper->GetAngle();
}

void ExtendHopper::Execute() {
  m_currentHopperAngle += 0.01;

  m_hopper->SetAngle(m_currentHopperAngle);
 // double power;

//  if(m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightTrigger) > 0.0) {
//  m_hopper->SetHopperMotorPower(-0.3);
  //power = -0.3;
  //}

//  else if(m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftTrigger) > 0.0) {
//  m_hopper->SetHopperMotorPower(0.3);
//  //power = 0.3;
//  }
//  else {
//    m_hopper->SetHopperMotorPower(0.0);
//  }
  //m_hopper->SetHopperMotorPower(power);
}


void ExtendHopper::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_hopper->SetHopperMotorPower(0.0);
  }
#endif