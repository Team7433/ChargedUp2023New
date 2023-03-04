// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithJoystick.h"

using namespace JoystickDriveConstants;

DriveWithJoystick::DriveWithJoystick(SwerveDrivetrain* swerveDriveTrain, frc::Joystick* joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDriveTrain);

  m_joystick = joystick;
  m_swerveDrivetrain = swerveDriveTrain;

}

// Called when the command is initially scheduled.
void DriveWithJoystick::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute() {

  const double k_joystickDeadzone{0.03};
  double maxVelocity{3.0};
  double maxRotationVelocity{0.9};

  if(m_joystick->GetRawButton(1)) {
    maxVelocity = maxVelocity/4;
    maxRotationVelocity = maxRotationVelocity/2;
  }

  m_swerveDrivetrain->Drive(units::radian_t(-m_joystick->GetDirectionRadians()), units::meter_t( (m_joystick->GetMagnitude() > k_joystickDeadzone) ? pow(m_joystick->GetMagnitude(), 2) * maxVelocity : 0) ,(fabs(m_joystick->GetZ()) > k_joystickDeadzone) ? ((std::signbit(m_joystick->GetZ())) ? -1 : 1) * pow(m_joystick->GetZ(), 2)*maxRotationVelocity : 0);

}

// Called once the command ends or is interrupted.
void DriveWithJoystick::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystick::IsFinished() {
  return false;
}