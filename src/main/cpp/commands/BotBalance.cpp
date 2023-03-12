// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BotBalance.h"
// TODO no auto

BotBalance::BotBalance(SwerveDrivetrain * swerve, Gyro * gyro) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerve});
  m_gyro = gyro;
  m_swerveDrive = swerve;
}

// Called when the command is initially scheduled.
void BotBalance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BotBalance::Execute() {

  double rotMagnitude = getRotationError().to<double>() * 0.1;
  std::cout << "Roll: " << m_gyro->GetRoll().to<double>() << " ";

  if (m_gyro->GetRoll() < -m_threshold){
    m_swerveDrive->Drive(units::radian_t(0), 0.15_m, 0); // do things
    std::cout << "moving Backward\n";
    m_timer.Stop();
    m_timer.Reset();
  }
   else if (m_gyro->GetRoll() > m_threshold){
    m_swerveDrive->Drive(units::radian_t(pi), 0.15_m, 0);
    std::cout << "Moving forward\n";
    m_timer.Stop();
    m_timer.Reset();

  } else {
    m_timer.Start();
    m_swerveDrive->Drive(units::radian_t(pi/2), 0.0_m, 0);
    if(m_timer.Get() > 0.5_s) {
      m_done = true;
    }
  
  } 
}

// Called once the command ends or is interrupted.
void BotBalance::End(bool interrupted) {

  m_done = false;
  m_timer.Stop();
  m_timer.Reset();

}

// Returns true when the command should end.
bool BotBalance::IsFinished() {
  return m_done; // TODO
}



units::degree_t BotBalance::getRotationError() {

  double signOfGyroAngle = abs(m_gyro->GetRotation())/m_gyro->GetRotation();

  return closestPathError(180_deg - units::degree_t(signOfGyroAngle==1?m_gyro->GetRotation():m_gyro->GetRotation()+360));

}


units::radian_t BotBalance::closestPathError(units::radian_t error) {

    if (units::math::fabs(error) <= 180_deg) return error; // if the error is smaller than 90 deg do nothing

    double m_currentDirection = units::math::fabs(error)/error; // the sign of the error -1 or 1 
    
    units::radian_t newError = (360_deg - units::math::fabs(error)); // find the error required to turn in the other direction
    
    newError = newError * -m_currentDirection; //make the sign of the new error to the opposite of the old error

    return newError; 
}
