// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm(){
    m_armMotorOne->SetNeutralMode(NeutralMode::Brake);
    m_armMotorTwo->SetNeutralMode(NeutralMode::Brake);
    m_armMotorTwo->Follow(*m_armMotorOne);

    m_armMotorOne->Config_kP(0, 0.02, 10);
    m_armMotorOne->Config_kI(0, 0, 10);
    m_armMotorOne->Config_kD(0, 0.2, 10);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
    frc::SmartDashboard::PutNumber("ArmPos", m_armMotorOne->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("ArmTraj", m_armMotorOne->GetClosedLoopTarget());
    // frc::SmartDashboard::PutNumber("ArmError", m_armMotorOne->GetClosedLoopError());
    // std::cout << m_armMotorOne->GetMotorOutputPercent()<< std::endl;
    // frc::SmartDashboard::PutNumber("ArmAngle", getAngle());


}
