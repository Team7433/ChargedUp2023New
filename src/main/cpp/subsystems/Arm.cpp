// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm(){
    enableBrakeMode();

    m_armMotorTwo->Follow(*m_armMotorOne);

    m_armMotorOne->Config_kP(0, 0.3, 10);
    m_armMotorOne->Config_kI(0, 0.0, 10);
    m_armMotorOne->Config_kD(0, 0.2, 10);

    //motion 
    m_armMotorOne->ConfigMotionAcceleration(3500, 10);
    m_armMotorOne->ConfigMotionCruiseVelocity(9000, 10);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
    frc::SmartDashboard::PutNumber("Arm/Position", m_armMotorOne->GetSelectedSensorPosition());

    if(m_armMotorOne->GetControlMode() == ControlMode::Position || m_armMotorOne->GetControlMode() == ControlMode::MotionMagic) {
        frc::SmartDashboard::PutNumber("Arm/TargetPosition", m_armMotorOne->GetClosedLoopTarget());
        frc::SmartDashboard::PutNumber("Arm/Error", m_armMotorOne->GetClosedLoopError());
    }

    if (m_armMotorOne->GetControlMode() == ControlMode::MotionMagic) {
        frc::SmartDashboard::PutNumber("Arm/MotionMagicTarget", getActiveTrajectoryPosition());
        frc::SmartDashboard::PutBoolean("Arm/MotionMagicComplete", m_motionMagicTarget == getActiveTrajectoryPosition());
    }
    

}

void Arm::setPercentageOutput(double output) {
    m_armMotorOne->Set(ControlMode::PercentOutput, output);
}



void Arm::setMotionMagic(double position) {

    if (position < -80000 || position > 0) { std::cout << "Aborted Run Motion Magic to pos: " << position << ", out of safety limit!\n"; return;}
    enableBrakeMode();
    m_motionMagicTarget = position;
    m_armMotorOne->Set(ControlMode::MotionMagic, position);


}

double Arm::getActiveTrajectoryPosition() {
    return m_armMotorOne->GetActiveTrajectoryPosition();
}


double Arm::getPosition() {
    return m_armMotorOne->GetSelectedSensorPosition();
}

double Arm::getTargetPos() {

    return m_armMotorOne->GetClosedLoopTarget();
}

void Arm::extendArm(frc::DoubleSolenoid::Value val) {
    extensionSolenoid.Set(val);
}

double Arm::getMotionMagicTargetPosition() {
    return m_motionMagicTarget;
}

void Arm::freeArm() {
    m_armMotorOne->SetNeutralMode(motorcontrol::NeutralMode::Coast);
    m_armMotorTwo->SetNeutralMode(motorcontrol::NeutralMode::Coast);


    m_armMotorOne->Set(ControlMode::PercentOutput, 0.0);

}

void Arm::enableBrakeMode() {
    m_armMotorOne->SetNeutralMode(NeutralMode::Brake);
    m_armMotorTwo->SetNeutralMode(NeutralMode::Brake);
    
}