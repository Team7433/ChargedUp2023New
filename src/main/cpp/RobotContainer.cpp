// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() : m_swerveDrive(&m_gyro){
  // Initialize all of your commands and subsystems here
  m_swerveDrive.SetDefaultCommand(DriveWithJoystick(&m_swerveDrive, &m_joystick));
  m_arm.SetDefaultCommand(JoystickArmControl(&m_arm, &m_driverController));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  frc2::Trigger([this] {return m_joystick.GetRawButton(3);}).OnTrue(MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 0_deg).ToPtr());



  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_joystick.GetRawButton(2);
  }).OnTrue(frc2::InstantCommand([this]{m_gyro.Reset(); m_swerveDrive.ResetOdometry();}).ToPtr());

  // frc2::Trigger([this]{
  //   return m_joystick.GetRawButton(1);
  // }).OnTrue(AutoTarget(&m_swerveDrive, &m_gyro, &m_vision, &m_joystick, &m_joystick).ToPtr());


  // frc2::Trigger([this]{
  //   return m_joystick.GetRawButton(4);
  // }).OnTrue(frc2::SequentialCommandGroup(

  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -1.55_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5.2_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = -2.25_m}, 0_deg),
  //   MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 90_deg)

  // ).ToPtr());




  frc2::Trigger([this]{
    return m_joystick.GetRawButton(4);
  }).OnTrue(frc2::SequentialCommandGroup(
    SetArmPosition(&m_arm, -60000),
    frc2::InstantCommand([this]{m_claw.setClaw(frc::DoubleSolenoid::kReverse);}),
    MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 5_m, .y_pos = 0.0_m}, 0_deg)


  ).ToPtr());


  // frc2::Trigger([this]{return m_driverController.GetRawButton(8);}).OnTrue(TurnToTarget(&m_swerveDrive, &m_gyro, &m_vision).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.

  m_driverController.A().WhileTrue(frc2::InstantCommand([this]{m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());
  m_driverController.Y().WhileTrue(frc2::InstantCommand([this]{m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());

  

  // m_driverController.Start().WhileTrue(HoldArm(&m_arm, &m_controller).ToPtr());
  // m_driverController.X().WhileTrue(frc2::InstantCommand([this]{m_arm.setPosition(m_arm.getTargetPos()-2000);}).ToPtr());
  // m_driverController.B().WhileTrue(frc2::InstantCommand([this]{m_arm.setPosition(m_arm.getTargetPos()+2000);}).ToPtr());

  m_driverController.X().WhileTrue(frc2::InstantCommand([this]{m_arm.RunMotionMagic(-78000);}).ToPtr());
  // m_driverController.B().WhileTrue(frc2::InstantCommand([this]{m_arm.RunMotionMagic(-50000);}).ToPtr());
  m_driverController.B().WhileTrue(SetArmPosition(&m_arm, -50000).ToPtr());

  m_driverController.RightBumper().WhileTrue(frc2::InstantCommand([this]{m_claw.setClaw(frc::DoubleSolenoid::kForward);}).ToPtr());
  m_driverController.LeftBumper().WhileTrue(frc2::InstantCommand([this]{m_claw.setClaw(frc::DoubleSolenoid::kReverse);}).ToPtr());



  // TODO configure some sort of command that when the Y button is pressed will move the arm to the controller axis

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
