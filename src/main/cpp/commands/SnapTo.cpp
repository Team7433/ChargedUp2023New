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
void SnapTo::Initialize() {

  double minDifference = 10000000;
  std::string closestPosition;

  double currentArmPos = m_arm->getPosition();

  for (auto const& [key, val] : armPositions){
    double currentDiff = fabs(val - currentArmPos);
    if (currentDiff < minDifference){
      minDifference = currentDiff;
      closestPosition = key;
    }
    
  }

  // Turns out I could've used a while loop, but forgor to increment :p

  m_target = armPositions[closestPosition];
  m_arm->setMotionMagic(armPositions[closestPosition]); // Set to closest position;
  std::cout << "AutoSnap: " << closestPosition << " " << armPositions[closestPosition] << std::endl;

  }


// Called repeatedly when this Command is scheduled to run
void SnapTo::Execute() {
  // if (isControllerActive()){ // If controller is active, do manual control
  //   //m_arm->setMotionMagic(std::copysign(pow(m_controller->GetLeftY(), 2), -m_controller->GetLeftY())*4000 + m_arm->getPosition());
  //   // std::cout << "Manual Control: " << std::copysign(pow(m_controller->GetLeftY(), 2), -m_controller->GetLeftY())*4000 + m_arm->getPosition() << std::endl;
  //   // return;
  // }


  

}

// Called once the command ends or is interrupted.
void SnapTo::End(bool interrupted) {
  std::cout << "Done" << std::endl;
}

// Returns true when the command should end.
bool SnapTo::IsFinished() {
  return (m_target == m_arm->getActiveTrajectoryPosition());
}

bool SnapTo::isControllerActive(){
  return ((m_controller->GetLeftY() > 0.02) || (m_controller->GetLeftY() < -0.02));
}
