// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnToTarget.h"

TurnToTarget::TurnToTarget(SwerveDrivetrain* swerveDriveTrain, Gyro* gyro, Vision* vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerveDriveTrain});

  m_swerveDrive= swerveDriveTrain;
  m_gyro = gyro;
  m_vision = vision;
}

// Called when the command is initially scheduled.
void TurnToTarget::Initialize() {
  std::cout << "TurnToTarget"<< std::endl;
  m_gyroTarget = units::degree_t(m_gyro->GetRotation()) + m_vision->getTargetOffsetX();
}

// Called repeatedly when this Command is scheduled to run
void TurnToTarget::Execute() {
  if(m_vision->getTargetVisible() && m_vision->getTXUpToDate()){
    m_gyroTarget = units::degree_t(m_gyro->GetRotation()) + m_vision->getTargetOffsetX();
  }

  //sets the error for rotation
  m_error = getCLosestError(m_gyroTarget);

  //KI controller Loop
  if (fabs(m_error.to<double>()) < m_izone) {
    m_accumulator += m_error.to<double>();
  } else {
    m_accumulator = 0.0;
  }
  if (m_accumulator >= m_maxAccumulator) {
    m_accumulator = m_maxAccumulator;
  }
  //rotate output variable
  double rotate{m_error.to<double>()*kPID["kP"] + m_accumulator*kPID["kI"] + kPID["kS"] };

  // std::cout << "error: " << m_error.to<double>() << " output: " << rotate << std::endl;
  //telling swerveDrive Controller to drive with the above outputs
  m_swerveDrive->Drive(units::angle::radian_t(0), units::length::meter_t(0.0), rotate, units::degree_t(m_gyro->GetRotation()));
}

// Called once the command ends or is interrupted.
void TurnToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool TurnToTarget::IsFinished() {
  if (units::degree_t(fabs(getCLosestError(m_gyroTarget).to<double>())) < 2_deg){return true;}
}

units::degree_t TurnToTarget::getCLosestError(units::degree_t targetGyroAngle) {

  targetGyroAngle = getRemappedGyroAngle(targetGyroAngle);

  units::degree_t error{0_deg};
  if (units::math::fabs(targetGyroAngle - units::degree_t(m_gyro->GetRotation())) <= 180_deg - units::math::fabs(units::degree_t(m_gyro->GetRotation()) + 180_deg - units::math::fabs(targetGyroAngle))) {
    error = units::degree_t(targetGyroAngle.to<double>() - m_gyro->GetRotation());

  } else {
    error = units::math::fabs(180_deg - units::degree_t(m_gyro->GetRotation())) + units::math::fabs(180_deg - targetGyroAngle);

  }

  return error;

}

units::degree_t TurnToTarget::getRemappedGyroAngle(units::degree_t targetGyroAngle) {

  if (targetGyroAngle <= -180_deg) {
    targetGyroAngle = 180_deg-units::degree_t(remainderf(-(targetGyroAngle.to<double>()), 180));
  } else if (targetGyroAngle >= 180_deg) {
    targetGyroAngle = 180_deg-units::degree_t(remainderf((targetGyroAngle.to<double>()), 180));
  }
  return targetGyroAngle;

}