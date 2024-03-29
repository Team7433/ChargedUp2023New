// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/JoystickArmControl.h"

JoystickArmControl::JoystickArmControl(Arm* arm, frc2::CommandXboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);

  m_arm = arm;
  m_joystick = controller;

}

// Called when the command is initially scheduled.
void JoystickArmControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void JoystickArmControl::Execute() {
  if( fabs(m_joystick->GetLeftY()) < 0.02) return; // TODO fix this as the controller axis can be negative
  // m_arm->setArm(std::copysign(pow(m_joystick->GetLeftY(), 2), -m_joystick->GetLeftY())*0.17);
  m_arm->setMotionMagic(std::copysign(pow(m_joystick->GetLeftY(), 2), -m_joystick->GetLeftY())*7000 + m_arm->getPosition());

}

// Called once the command ends or is interrupted.
void JoystickArmControl::End(bool interrupted) {}

// Returns true when the command should end.
bool JoystickArmControl::IsFinished() {
  return false;
}
