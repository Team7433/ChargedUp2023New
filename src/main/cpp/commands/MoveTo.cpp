// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveTo.h"

MoveTo::MoveTo(SwerveDrivetrain* SwerveDrive, Gyro* gyro, iona::coordinate targetCoordinate, units::degree_t faceDirection) {
  AddRequirements(SwerveDrive);

  m_gyro = gyro;
  m_swerveDrive = SwerveDrive;

  m_targetCoordinate = targetCoordinate;

  m_targetFaceDirection = faceDirection;

}

MoveTo::MoveTo(SwerveDrivetrain* SwerveDrive, Gyro* gyro, iona::coordinate targetCoordinate, units::degree_t faceDirection, MoveToConfig config) {
  AddRequirements(SwerveDrive);

  m_gyro = gyro;
  m_swerveDrive = SwerveDrive;

  m_targetCoordinate = targetCoordinate;

  m_targetFaceDirection = faceDirection;

  m_maxVelocity = config.maxVelocity;
  m_maxAcceleration = config.Acceleration;



}

// Called when the command is initially scheduled.
void MoveTo::Initialize() {

  // std::cout << "Travel Direction To Coordinate: " << getMoveDirection().to<double>()*180/M_PI << " Degrees\n";
  std::cout << "Traveling to x: " << m_targetCoordinate.x_pos.to<double>() << ", y: " << m_targetCoordinate.y_pos.to<double>() << " from x: " << m_swerveDrive->getOdometryCoordinate().x_pos.to<double>() << ", y: " << m_swerveDrive->getOdometryCoordinate().y_pos.to<double>() << std::endl;

}

// Called repeatedly when this Command is scheduled to run
void MoveTo::Execute() {

  units::second_t deltaTime = getDeltaTime();

  m_newVelocity = units::math::min(

    m_maxVelocity,
    units::math::min(

      m_currentVelocity + m_maxAcceleration * deltaTime,
      units::math::sqrt(2 * m_maxAcceleration * getDistanceLeft())


    )
  );

  m_currentVelocity = m_newVelocity;




  // Motion profile for angles
  m_newAngleVelocity = units::math::min(

    m_maxAngleVelocity,
    units::math::min(

      m_currentAngleVelocity + m_angleAcceleration * deltaTime,
      units::math::sqrt(2 * m_angleAcceleration * units::math::fabs(units::meter_t(getRotationError().to<double>()))/100)


    )
  );

  m_currentAngleVelocity = m_newAngleVelocity;



  if (units::math::fabs(getRotationError()) < 10_deg) {
    m_accumulator -= getRotationError().to<double>();
  } else {
    m_accumulator = 0;
  }

  // std::cout << m_accumulator <<" " <<  m_gyro->GetRotation() << std::endl;
  // double angleRotationOutput = m_rotateKP*-(getRotationError()).to<double>() + m_rotateKI * m_accumulator;


  // std::cout << "Angle Velocity: " << -std::copysign(m_currentAngleVelocity.to<double>(), getRotationError().to<double>()) << " Rotation Error: " << getRotationError().to<double>() << std::endl;

  if(getDistanceLeft() < 0.02_m) {m_currentVelocity = 0_mps; m_newVelocity = 0_mps; }

  m_swerveDrive->Drive(getMoveDirection(), units::meter_t(m_newVelocity.to<double>()), -std::copysign(m_currentAngleVelocity.to<double>(), getRotationError().to<double>()));












}

// Called once the command ends or is interrupted.
void MoveTo::End(bool interrupted) {



  m_currentVelocity = 0_mps;
  m_newVelocity = 0_mps;

  m_currentAngleVelocity = 0_mps;
  m_newAngleVelocity = 0_mps;

  m_accumulator = 0;
  m_swerveDrive->Drive(0_rad, 0_m, 0.0);

  std::cout << "Finished Move To with distance error of " << getDistanceLeft().to<double>() << ", and angle error of " << getRotationError().to<double>() << " Degrees";

  m_timer.Stop();
  m_timer.Reset();

}

// Returns true when the command should end.
bool MoveTo::IsFinished() {

  bool distanceArrived = getDistanceLeft() < 0.02_m;
  bool angleArrived = units::math::fabs(getRotationError()) < 2_deg;

  return distanceArrived && angleArrived;
}

units::degree_t MoveTo::getRotationError() {

  double signOfGyroAngle = abs(m_gyro->GetRotation())/m_gyro->GetRotation();

  return closestPathError(m_targetFaceDirection - units::degree_t(signOfGyroAngle==1?m_gyro->GetRotation():m_gyro->GetRotation()+360));

}


units::radian_t MoveTo::closestPathError(units::radian_t error) {

    if (units::math::fabs(error) <= 180_deg) return error; // if the error is smaller than 90 deg do nothing

    double m_currentDirection = units::math::fabs(error)/error; // the sign of the error -1 or 1 
    
    units::radian_t newError = (360_deg - units::math::fabs(error)); // find the error required to turn in the other direction
    
    newError = newError * -m_currentDirection; //make the sign of the new error to the opposite of the old error

    return newError; 
}


units::radian_t MoveTo::getMoveDirection() {

  return units::math::atan2(m_targetCoordinate.y_pos - m_swerveDrive->getOdometryCoordinate().y_pos, m_targetCoordinate.x_pos - m_swerveDrive->getOdometryCoordinate().x_pos);

}

units::meter_t MoveTo::getDistanceLeft() {
  iona::coordinate currentCoord = m_swerveDrive->getOdometryCoordinate();
  return units::math::hypot(m_targetCoordinate.x_pos - currentCoord.x_pos, m_targetCoordinate.y_pos - currentCoord.y_pos);

}

units::second_t MoveTo::getDeltaTime() {

    m_timer.Stop();
    units::second_t timePassed = m_timer.Get();
    m_timer.Reset();
    m_timer.Start();
    return timePassed;
}