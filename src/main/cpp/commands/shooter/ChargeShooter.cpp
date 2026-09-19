
#include "Constants.h"
#include "commands/shooter/ChargeShooter.h"
#include "frc/Timer.h"
#include <units/time.h>

#ifndef CHASSIS_ONLY
ChargeShooter::ChargeShooter(ShooterSubsystem *shooter) : m_shooter{shooter} {
  AddRequirements(m_shooter);
}

void ChargeShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Initialized\r\n";
#endif
  m_shooter->SetShooterMotorPower(-0.9);

}

bool ChargeShooter::IsFinished(){
  
  units::second_t timestamp_secondsStart = frc::Timer::GetFPGATimestamp();
  if (double(timestamp_secondsStart) > 3.0){
    return true;
  }
  else {
    m_shooter->SetShooterMotorPower(-0.9);
  }
}
void ChargeShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ChargeShooter Ended\r\n";
#endif
  m_shooter->SetShooterMotorPower(0.0);
}
#endif