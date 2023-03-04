// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GridAlign.h"


GridAlign::GridAlign(Gyro * gyro, SwerveDrivetrain * swerve, Vision * vision, Arm * arm) {
  // Use addRequirements() here to declare subsystem dependencies

  // Setting all our member variables to pointers of subsystem instances.
  m_gyro = gyro;
  m_swerve = swerve;
  m_vision = vision;
  m_arm = arm;

}

// Called when the command is initially scheduled.
void GridAlign::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void GridAlign::Execute() {
  MoveTo(m_swerve, m_gyro, m_swerve->getOdometryCoordinate(), 180_deg).Schedule();
  double sign = m_vision->getTargetOffsetX().to<double>()/fabs(m_vision->getTargetOffsetX().to<double>()); // get the direction -1 or 1
  units::radian_t direction = (fabs(sign) == sign) ? units::radian_t(M_PI/2) : units::radian_t(3*M_PI/2);  

  units::meter_t magnitude = sin(m_vision->getTargetOffsetX().to<double>()) * m_vision->getTargetPixelArea() * pow(VisionConstants::kPixelCount, -1) * 1_m;
  m_swerve->Drive(direction, magnitude, 0);

  
  
}

// Called once the command ends or is interrupted.
void GridAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool GridAlign::IsFinished() {
  return false;
}
