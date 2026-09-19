
#include "Constants.h"
#include "commands/hopper/MoveHopperWithTriggers.h"

#ifndef CHASSIS_ONLY
MoveHopperWithTriggers::MoveHopperWithTriggers(HopperSubsystem *hopper, frc::XboxController *operatorController) : m_hopper{hopper}, m_operatorController{operatorController} {
  AddRequirements(m_hopper);
}

void MoveHopperWithTriggers::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_hopper->SetAngle(HopperConstants::kHopperMinAngle);
m_currentHopperAngle = m_hopper->GetAngle();

}

void MoveHopperWithTriggers::Execute() {

double power;

  if(m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightTrigger) > 0.0) {
    // m_hopper->SetHopperMotorPower(-0.3);
    m_currentHopperAngle -= 0.005;
  } else if(m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftTrigger) > 0.0) {
//  m_hopper->SetHopperMotorPower(0.3);
    m_currentHopperAngle += 0.01;
  } else if (m_hopper->GetAngle() < (((HopperConstants::kHopperMaxAngle - HopperConstants::kHopperMinAngle) / 2) + HopperConstants::kHopperMinAngle)) {
    m_hopper->SetHopperMotorPower(0.0);
    return;
  } 
  if(m_currentHopperAngle > HopperConstants::kHopperMaxAngle) {
    m_currentHopperAngle = HopperConstants::kHopperMaxAngle;
 }
  if(m_currentHopperAngle < HopperConstants::kHopperMinAngle) {
    m_currentHopperAngle = HopperConstants::kHopperMinAngle;
  }

  m_hopper->SetAngle(m_currentHopperAngle);
}

void MoveHopperWithTriggers::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_hopper->SetHopperMotorPower(0.0);
  }
#endif