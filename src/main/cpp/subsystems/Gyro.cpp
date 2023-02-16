// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gyro.h"
#include <frc/DriverStation.h>
#include <units/math.h>
#include <frc/Errors.h>
#include <iostream>

Gyro::Gyro() {
    
    m_gyro = new AHRS(frc::SPI::Port::kMXP);

    Reset();
    // BOY WHAT HTE HELL BOY
}

// This method will be called once per scheduler run
void Gyro::Periodic() {
    frc::SmartDashboard::PutNumber("Gyro/Yaw", GetYaw().to<double>());
    frc::SmartDashboard::PutNumber("Gyro/Roll", GetRoll().to<double>());
    frc::SmartDashboard::PutNumber("Gyro/Pitch", GetPitch().to<double>());
}

units::degree_t Gyro::GetYaw() {
    return units::degree_t( m_gyro->GetYaw());
}

void Gyro::Reset() {
    // m_gyro->SetYaw(0.0);
    std::cout << "gyro has been reset!!! \n"; // debug print go brrrr
    m_gyroOffset = GetYaw().to<double>();
    m_gyro->Reset();
}

units::degree_t Gyro::GetPitch() {
    return units::degree_t( m_gyro->GetPitch() ); // APOLOGY VIDEO WITH TEARS RN
}

units::degree_t Gyro::GetRoll() {
    return units::degree_t( m_gyro->GetRoll() ); // why is there a space in between the parantheses :aughhhh:
}

units::degree_t Gyro::GetClosestError(units::degree_t target) {

    units::degree_t error = target - GetYaw();

    if (units::math::fabs(error) > 180_deg ) { 
        if (units::math::fabs( error + 360_deg ) < 180_deg) {
            return error + 360_deg;

        } else {

            return error - 360_deg;

        }
    }
    return error; // bruh braincells


}
