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

  // -- AUTOS --
  frc::SmartDashboard::PutData(&autoSelecter);
  autoSelecter.AddOption("2Right", 1); // Red
  autoSelecter.AddOption("1MidChargeStation", 2);
  autoSelecter.AddOption("2RightB", 3); // Blue
  autoSelecter.AddOption("LeaveCommunity", 4);
  autoSelecter.AddOption("NoAuto", 5);
  autoSelecter.AddOption("1ItemEdge", 6); 
  // -- AUTOS --

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  // frc2::Trigger([this] {return m_driverStick.GetRawButton(3);}).OnTrue(MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 0_deg).ToPtr());

  frc2::Trigger([this] {
    return m_driverStick.GetRawButton(3);
  }).OnTrue(BotBalance(&m_swerveDrive, &m_gyro).ToPtr());



  // frc2::Trigger([this]{
  //   return m_driverStick.GetRawButton(1);
  // }).OnTrue(AutoTarget(&m_swerveDrive, &m_gyro, &m_vision, &m_driverStick, &m_driverStick).ToPtr());





  //Swerve drive gyroscope and odometry reset.
  frc2::Trigger([this] {
    return m_driverStick.GetRawButton(2);
  }).OnTrue(frc2::InstantCommand([this]{m_gyro.Reset(); m_swerveDrive.ResetOdometry();}).ToPtr());

  //Zeroing (resetting) swerve modules.
  frc2::Trigger([this] {return m_driverStick.GetRawButton(5);}).OnTrue(
    frc2::InstantCommand([this] {m_swerveDrive.ResetSwerveModules();}).ToPtr()
  );
  //////// CODRIVER BINDINGS /////////


  // Robot arm claw control
  m_controller.LeftBumper().WhileTrue(frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);}).ToPtr());
  m_controller.RightBumper().WhileTrue(frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}).ToPtr());


  m_controller.Start().WhileTrue(frc2::InstantCommand([this] {m_arm.freeArm();}).ToPtr());


  //
  // ---- NEW CODRIVER BINDINGS
  //


  //Arm telescoping control.
  m_controller.A().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());
  m_controller.Y().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());

  m_controller.X().WhileTrue(SnapTo(&m_arm, &m_controller).ToPtr()); // Snap to the closest known position



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  switch (autoSelecter.GetSelected()){
    case 1: return TwoItemAutoRight(&m_arm, &m_swerveDrive, &m_gyro).ToPtr();
    case 2: return OneItemMid(&m_arm, &m_swerveDrive, &m_gyro).ToPtr();
    case 3: return TwoItemAutoRightTwo(&m_arm, &m_swerveDrive, &m_gyro).ToPtr();
    case 4: return LeaveCommunityLeft(&m_swerveDrive, &m_gyro).ToPtr();
    case 5: return frc2::CommandPtr{nullptr};
  } 

  return frc2::CommandPtr{nullptr};

}
