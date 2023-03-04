// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Wait.h"

Wait::Wait(units::second_t length) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_length = length;
}

// Called when the command is initially scheduled.
void Wait::Initialize() {
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void Wait::Execute() {}

// Called once the command ends or is interrupted.
void Wait::End(bool interrupted) {
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool Wait::IsFinished() {
  return m_timer.Get() > m_length;
}
