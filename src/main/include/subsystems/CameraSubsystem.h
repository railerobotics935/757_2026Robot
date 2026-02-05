
#pragma once

#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DigitalInput.h>
#include <photon/PhotonCamera.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/SparkMax.h>
#include <iostream>
#include <Constants.h>

class CameraSubsystem : public frc2::SubsystemBase {
    public:
    
    CameraSubsystem();

    
    
    void Periodic() override;

    private:
    
    photon::PhotonCamera camera{"Camera1"};


};