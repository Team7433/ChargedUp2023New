// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrivetrain.h"

SwerveDrivetrain::SwerveDrivetrain(Gyro* gyro){
    m_gyro = gyro;
}

// This method will be called once per scheduler run
void SwerveDrivetrain::Periodic() {
    m_swerveDrive->updateOdometry(units::radians_per_second_t(m_gyro->GetRotationChange() * (M_PI/180)));
    // m_swerveModuleFR->displayModuleData();
    // m_swerveModuleFL->displayModuleData();
    // m_swerveModuleBR->displayModuleData();
    // m_swerveModuleBL->displayModuleData();
    m_swerveDrive->DisplayData();
}


void SwerveDrivetrain::Drive(units::radian_t direction, units::meter_t magnitude, double rotation, units::radian_t gyroAngle) {
    iona::Vector2D directionVec(direction, magnitude);

    m_swerveDrive->Drive(directionVec, rotation, units::degree_t(m_gyro->GetRotation()));
    //std::cout << units::degree_t(->GetRotation() << std::endl;
}

