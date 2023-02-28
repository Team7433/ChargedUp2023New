// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveTo.h"

MoveTo::MoveTo(SwerveDrivetrain* SwerveDrive, Gyro* gyro, iona::coordinate targetCoordinate, units::degree_t faceDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(SwerveDrive);

  m_gyro = gyro;
  m_swerveDrive = SwerveDrive;

  m_targetCoordinate = targetCoordinate;

  m_targetFaceDirection = faceDirection;



}

// Called when the command is initially scheduled.
void MoveTo::Initialize() {

  std::cout << "Travel Direction To Coordinate: " << getMoveDirection().to<double>()*180/M_PI << " Degrees\n";

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


  double angleRotationOutput = m_rotateKP*(getRotationError()).to<double>();

  m_swerveDrive->Drive(getMoveDirection(), units::meter_t(m_newVelocity.to<double>()), angleRotationOutput);












}

// Called once the command ends or is interrupted.
void MoveTo::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveTo::IsFinished() {

  bool distanceArrived = getDistanceLeft() < 0.2_m;
  bool angleArrived = getRotationError() < 3_deg;

  return distanceArrived && angleArrived;
}

units::degree_t MoveTo::getRotationError() {

  return m_targetFaceDirection - units::degree_t(m_gyro->GetRotation());

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