// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SnapTo.h"

SnapTo::SnapTo(Arm * arm, frc2::CommandXboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
  m_arm = arm;
  m_controller = controller;
}

// Called when the command is initially scheduled.
void SnapTo::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SnapTo::Execute() {}

// Called once the command ends or is interrupted.
void SnapTo::End(bool interrupted) {}

// Returns true when the command should end.
bool SnapTo::IsFinished() {
  return false;
}

bool SnapTo::isControllerActive(){
  return true; // TODO
}
