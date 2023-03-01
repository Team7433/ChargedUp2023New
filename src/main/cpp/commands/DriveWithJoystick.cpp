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
  // m_swerveDrivetrain->setHeading(units::radian_t(-m_joystick->GetDirectionRadians()));
  // std::cout << -m_joystick->GetDirectionRadians() << std::endl;
  // m_swerveDrivetrain->setOutput(m_joystick->GetMagnitude()*0.1);


  m_swerveDrivetrain->Drive(units::radian_t(-m_joystick->GetDirectionRadians()), units::meter_t((m_joystick->GetMagnitude()>0.06)?pow(m_joystick->GetMagnitude(), 2)*3.5:0) , m_joystick->GetZ()*0.9);


}

// Called once the command ends or is interrupted.
void DriveWithJoystick::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystick::IsFinished() {
  return false;
}