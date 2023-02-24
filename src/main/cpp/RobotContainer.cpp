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

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_joystick.GetRawButton(2);
  }).OnTrue(frc2::InstantCommand([this]{m_gyro.Reset(); m_swerveDrive.ResetOdometry();}).ToPtr());

  frc2::Trigger([this]{
    return m_joystick.GetRawButton(1);
  }).OnTrue(TurnToTarget(&m_swerveDrive, &m_gyro, &m_vision).ToPtr());

  // frc2::Trigger([this]{return m_driverController.GetRawButton(8);}).OnTrue(TurnToTarget(&m_swerveDrive, &m_gyro, &m_vision).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.

  m_driverController.A().WhileTrue(frc2::InstantCommand([this]{m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());
  m_driverController.Y().WhileTrue(frc2::InstantCommand([this]{m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());
  

  m_driverController.Start().WhileTrue(HoldArm(&m_arm, &m_controller).ToPtr());
  // m_driverController.X().WhileTrue(frc2::InstantCommand([this]{m_arm.setPosition(m_arm.getTargetPos()-2000);}).ToPtr());
  // m_driverController.B().WhileTrue(frc2::InstantCommand([this]{m_arm.setPosition(m_arm.getTargetPos()+2000);}).ToPtr());
  // m_driverController.B().WhileTrue(frc2::InstantCommand([this]{m_arm.setArm(0.0);}).ToPtr());


  m_driverController.RightBumper().WhileTrue(frc2::InstantCommand([this]{m_claw.setClaw(frc::DoubleSolenoid::kForward);}).ToPtr());
  m_driverController.LeftBumper().WhileTrue(frc2::InstantCommand([this]{m_claw.setClaw(frc::DoubleSolenoid::kReverse);}).ToPtr());



  // TODO configure some sort of command that when the Y button is pressed will move the arm to the controller axis

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
