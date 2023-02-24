// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoldArm.h"

HoldArm::HoldArm(Arm* arm, frc::XboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
  m_arm = arm;
  m_controller = controller;
  
}

// Called when the command is initially scheduled.
void HoldArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoldArm::Execute() {
  m_arm->setPosition(m_arm->getPosition());
}

// Called once the command ends or is interrupted.
void HoldArm::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool HoldArm::IsFinished() {
  return m_controller->GetBButton();
}
