// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveTo.h"

MoveTo::MoveTo(SwerveDrivetrain* SwerveDrive, iona::coordinate targetCoordinate) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void MoveTo::Initialize() {

  std::cout << "Travel Direction To Coordinate: " << getMoveDirection().to<double>()*180/M_PI << " Degrees\n";

}

// Called repeatedly when this Command is scheduled to run
void MoveTo::Execute() {}

// Called once the command ends or is interrupted.
void MoveTo::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveTo::IsFinished() {
  return true;
}


units::radian_t MoveTo::getMoveDirection() {

  return units::math::atan2(m_targetCoordinate.y_pos - m_swerveDrive->getOdometryCoordinate().y_pos, m_targetCoordinate.x_pos - m_swerveDrive->getOdometryCoordinate().x_pos);

}

