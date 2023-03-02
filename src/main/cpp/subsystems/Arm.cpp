// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm(){
    m_armMotorOne->SetNeutralMode(NeutralMode::Brake);
    m_armMotorTwo->SetNeutralMode(NeutralMode::Brake);
    
    m_armMotorTwo->Follow(*m_armMotorOne);

    m_armMotorOne->Config_kP(0, 0.3, 10);
    m_armMotorOne->Config_kI(0, 0.0, 10);
    m_armMotorOne->Config_kD(0, 0.2, 10);

    //motion 
    m_armMotorOne->ConfigMotionAcceleration(1500, 10);
    m_armMotorOne->ConfigMotionCruiseVelocity(4000, 10);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
    frc::SmartDashboard::PutNumber("ArmPos", m_armMotorOne->GetSelectedSensorPosition());

    if(m_armMotorOne->GetControlMode() == ControlMode::Position || m_armMotorOne->GetControlMode() == ControlMode::MotionMagic) {
        frc::SmartDashboard::PutNumber("ArmTraj", m_armMotorOne->GetClosedLoopTarget());
        frc::SmartDashboard::PutNumber("ArmError", m_armMotorOne->GetClosedLoopError());
        std::cout << MotionMagicComplete() << std::endl;
    }
    

}
