// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() : m_swerveDrive(&m_gyro){
  // Initialize all of your commands and subsystems here
  m_swerveDrive.SetDefaultCommand(DriveWithJoystick(&m_swerveDrive, &m_driverStick));
  m_arm.SetDefaultCommand(JoystickArmControl(&m_arm, &m_controller));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  frc2::Trigger([this] {return m_driverStick.GetRawButton(3);}).OnTrue(MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 0_deg).ToPtr());




  // frc2::Trigger([this]{
  //   return m_driverStick.GetRawButton(1);
  // }).OnTrue(AutoTarget(&m_swerveDrive, &m_gyro, &m_vision, &m_driverStick, &m_driverStick).ToPtr());


  // frc2::Trigger([this]{
  //   return m_driverStick.GetRawButton(4);
  // }).OnTrue(frc2::SequentialCommandGroup(

  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -1.55_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 90_deg)

  // ).ToPtr());

  //test of autonomous command
  frc2::Trigger([this]{ return m_driverStick.GetRawButton(4); }).OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] {m_swerveDrive.ResetOdometry();}),
      SetArmPosition(&m_arm, -60000),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5_m, .y_pos = 0.0_m}, 0_deg)
    ).ToPtr());





  //reset gyro and odometry
  frc2::Trigger([this] {
    return m_driverStick.GetRawButton(2);
  }).OnTrue(frc2::InstantCommand([this]{m_gyro.Reset(); m_swerveDrive.ResetOdometry();}).ToPtr());

  //re zero's swerve module pivot motors
  frc2::Trigger([this] {return m_driverStick.GetRawButton(5);}).OnTrue(
    frc2::InstantCommand([this] {m_swerveDrive.ResetSwerveModules();}).ToPtr()
  );

  //unlocks the arm from brake mode
  m_controller.Start().WhileTrue(frc2::InstantCommand([this] {m_arm.freeArm();}).ToPtr());

  //extend arm control
  m_controller.A().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());
  m_controller.Y().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());

  //claw control
  m_controller.RightBumper().WhileTrue(frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);}).ToPtr());
  m_controller.LeftBumper().WhileTrue(frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}).ToPtr());

  //Arm move to collect cone position
  m_controller.X().WhileTrue(SetArmPosition(&m_arm, -78000).ToPtr());

  //Arm hold up cone
  m_controller.B().WhileTrue(SetArmPosition(&m_arm, -50000).ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::CommandPtr{nullptr};
}
