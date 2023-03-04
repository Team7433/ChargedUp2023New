// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SnapTo.h"

using namespace ArmConstants;

SnapTo::SnapTo(Arm * arm, frc2::CommandXboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
  m_arm = arm;
  m_controller = controller;
}

// Called when the command is initially scheduled.
void SnapTo::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SnapTo::Execute() {
  if (isControllerActive()){
    m_arm->setMotionMagic(std::copysign(pow(m_controller->GetLeftY(), 2), -m_controller->GetLeftY())*4000 + m_arm->getPosition());
    return;
  }

  auto iterator = armPositions.begin(); 

  double minDifference = 10000000;
  std::string closestPosition;

  double currentArmPos = m_arm->getPosition();

  while(iterator != armPositions.end()){
    double currentDiff = fabs(iterator->second - currentArmPos);
    if (currentDiff < minDifference){
      minDifference = currentDiff;
      closestPosition = iterator->first;
    }
  } 

  m_arm->setMotionMagic(armPositions[closestPosition]);

}

// Called once the command ends or is interrupted.
void SnapTo::End(bool interrupted) {}

// Returns true when the command should end.
bool SnapTo::IsFinished() {
  return false;
}

bool SnapTo::isControllerActive(){
  return ((m_controller->GetLeftY() < 0.02) && (m_controller->GetLeftY() > -0.02));
}
