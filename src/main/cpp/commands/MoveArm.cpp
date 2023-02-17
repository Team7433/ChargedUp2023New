// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveArm.h"

MoveArm::MoveArm(double angle, Arm* arm) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void MoveArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveArm::Execute() {}

// Called once the command ends or is interrupted.
void MoveArm::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveArm::IsFinished() {
  return false;
}
