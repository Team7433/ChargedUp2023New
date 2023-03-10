// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gyro.h"

Gyro::Gyro() = default;


// This method will be called once per scheduler run
void Gyro::Periodic() {

    // std::cout << "gyro " << GetRotationChange() << std::endl;

    // frc::SmartDashboard::PutNumber("Gyro Angle: ", GetRotation());
    // std::cout << "Roll" << GetRoll().to<double>() << std::endl;

}
