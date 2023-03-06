// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetArmPosition.h"

SetArmPosition::SetArmPosition(Arm* arm, double targetPosition) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(arm);


  m_arm = arm;
  m_targetPosition = targetPosition;



}

// Called when the command is initially scheduled.
void SetArmPosition::Initialize() {

  m_arm->setMotionMagic(m_targetPosition);
  std::cout << "Set Arm Position to " << m_targetPosition << " encoder counts " << std::endl;


}

// Called repeatedly when this Command is scheduled to run
void SetArmPosition::Execute() {

  // std::cout << m_arm->MotionMagicComplete() << std::endl;

}

// Called once the command ends or is interrupted.
void SetArmPosition::End(bool interrupted) {

  std::cout << "Arm arrived at " << m_targetPosition << " encoder counts " << std::endl;

}

// Returns true when the command should end.
bool SetArmPosition::IsFinished() {
  return m_arm->getActiveTrajectoryPosition() == m_targetPosition;
}
