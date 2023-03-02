// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTarget.h"


AutoTarget::AutoTarget(SwerveDrivetrain* swerveDriveTrain, Gyro* gyro, Vision* vision, frc::Joystick* joystick, frc::XboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerveDriveTrain});

  m_swerveDrive= swerveDriveTrain;
  m_gyro = gyro;
  m_vision = vision;
  m_joystick = joystick;
  m_controller = controller;
}

// Called when the command is initially scheduled.
void AutoTarget::Initialize() {
  std::cout <<"AutoTarget\n";
  if (m_vision->getTargetVisible())
  {
     m_gyroTarget = units::degree_t(m_gyro->GetRotation()) + m_vision->getTargetOffsetX();
  } else {
    m_done = true;
  }
  m_controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
  m_controller->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
}

// Called repeatedly when this Command is scheduled to run
void AutoTarget::Execute() {
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
  //driving forward variable
//   double forward = m_joystick->GetRawAxis(1);
//   if (fabs(forward) < kJoystickForwardDeadZone) {
//     forward = 0;
//   }
//   //strafe variable
//   double strafe = -m_joystick->GetRawAxis(0);
//   if (fabs(strafe) < kJoystickStrafeDeadZone) {
//     strafe = 0;
//   }
  // std::cout << "error: " << m_error.to<double>() << " output: " << rotate << std::endl;
  //telling swerveDrive Controller to drive with the above outputs
  m_swerveDrive->Drive(units::radian_t(-m_joystick->GetDirectionRadians()), units::meter_t(m_joystick->GetMagnitude()) , rotate);
}

// Called once the command ends or is interrupted.
void AutoTarget::End(bool interrupted) {
    m_done=false;
    m_controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
    m_controller->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
}

// Returns true when the command should end.
bool AutoTarget::IsFinished() {
  return !(m_joystick->GetRawButton(1));
}

units::degree_t AutoTarget::getCLosestError(units::degree_t targetGyroAngle) {

  targetGyroAngle = getRemappedGyroAngle(targetGyroAngle);

  units::degree_t error{0_deg};
  if (units::math::fabs(targetGyroAngle - units::degree_t(m_gyro->GetRotation())) <= 180_deg - units::math::fabs(units::degree_t(m_gyro->GetRotation())) + 180_deg - units::math::fabs(targetGyroAngle)) {
    error = targetGyroAngle - units::degree_t(m_gyro->GetRotation());

  } else {
    error =  units::math::fabs(180_deg - units::degree_t(m_gyro->GetRotation())) + units::math::fabs(180_deg - targetGyroAngle);

  }

  return error;

}

units::degree_t AutoTarget::getRemappedGyroAngle(units::degree_t targetGyroAngle) {

  if (targetGyroAngle <= -180_deg) {
    targetGyroAngle = 180_deg-units::degree_t(remainderf(-(targetGyroAngle.to<double>()), 180));
  } else if (targetGyroAngle >= 180_deg) {
    targetGyroAngle = 180_deg-units::degree_t(remainderf((targetGyroAngle.to<double>()), 180));
  }
  return targetGyroAngle;

}
