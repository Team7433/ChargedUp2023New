// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() : m_swerveDrive(&m_gyro){
  // Initialize all of your commands and subsystems here
  m_swerveDrive.SetDefaultCommand(DriveWithJoystick(&m_swerveDrive, &m_driverStick));
  
  // OLD CONTROLLER ARM CONTROL
  m_arm.SetDefaultCommand(JoystickArmControl(&m_arm, &m_controller));

  // SNAP TO POSITION ARM CONTROL
  // m_arm.SetDefaultCommand(SnapTo(&m_arm, &m_controller, false));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  frc2::Trigger([this] {return m_driverStick.GetRawButton(3);}).OnTrue(MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0_m, .y_pos = 0_m}, 0_deg).ToPtr());




  // frc2::Trigger([this]{
  //   return m_driverStick.GetRawButton(1);
  // }).OnTrue(AutoTarget(&m_swerveDrive, &m_gyro, &m_vision, &m_driverStick, &m_driverStick).ToPtr());


  frc2::Trigger([this]{ return m_driverStick.GetRawButton(7); }).OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_swerveDrive.ResetOdometry();}), // Reset Odometry
      SetArmPosition(&m_arm, -52000), // Set to above high cone position  
      frc2::InstantCommand([this]{ m_arm.extendArm(frc::DoubleSolenoid::kForward);}), // Extend arm
      Wait(0.2_s), // Wait for arm to extend
      SetArmPosition(&m_arm, -56000), // Place cone down into high
      frc2::InstantCommand([this]{ m_arm.setClaw(frc::DoubleSolenoid::kForward);}), // Release cone
      frc2::InstantCommand([this]{m_arm.extendArm(frc::DoubleSolenoid::kReverse);}), // Retract arm telescoping
      Wait(0.1_s), // Wait for arm to retract

      frc2::ParallelCommandGroup( // Set arm vertical and move simultaneously
      SetArmPosition(&m_arm, -48000), // Arm up
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0.5_m, .y_pos = -0.75_m}, 180_deg, MoveToConfig{.maxVelocity = 2.5_mps, .Acceleration = 2.5_mps_sq})),

      // Go near next cube
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.5_m, .y_pos = -0.75_m}, 0_deg),

      frc2::ParallelCommandGroup( // Put arm down and go to cube simultaneously
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.65_m, .y_pos = -0.25_m}, 0_deg),
      SetArmPosition(&m_arm, -83000)), // Arm to collect

      frc2::InstantCommand([this]{ m_arm.setClaw(frc::DoubleSolenoid::kReverse);}), // Collect cube

      frc2::ParallelCommandGroup( // Set arm up to above high cube position and move simultaneously
      SetArmPosition(&m_arm, -54000),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.65_m, .y_pos = -0.5_m}, 0_deg,  MoveToConfig{.maxVelocity = 1.5_mps, .Acceleration = 2.5_mps_sq})),
      
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0.2_m, .y_pos = -0.65_m}, 180_deg), // Move to grids
      SetArmPosition(&m_arm, -56000), // Set to high rung position  
      frc2::InstantCommand([this]{ m_arm.extendArm(frc::DoubleSolenoid::kForward);}), // Extend arm
      Wait(0.4_s), // Wait for arm to extend
      frc2::InstantCommand([this]{ m_arm.setClaw(frc::DoubleSolenoid::kForward);}) // Drop cone


      ).ToPtr());

  frc2::Trigger([this]{ return m_driverStick.GetRawButton(4); }).OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] {m_swerveDrive.ResetOdometry();}),
      SetArmPosition(&m_arm, -58000),
      frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::kForward);}),
      Wait(1.5_s),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);}),
      Wait(0.5_s),
      frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::kReverse);}),
      Wait(1.5_s),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}),
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(

        SetArmPosition(&m_arm, -1000),
        frc2::InstantCommand([this] {m_arm.freeArm();})

        ),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 2.55_m, .y_pos = 0.0_m}, 180_deg, MoveToConfig{ .maxVelocity = 0.4_mps, .Acceleration = 0.25_mps_sq})
      )


      


    ).ToPtr());




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





  //
  // ---- OLD (BUT WORKING) CODRIVER BINDINGS ----
  //


  // //Arm move to collect cone position
  // m_controller.A().WhileTrue(SetArmPosition(&m_arm, -83000).ToPtr());

  // //Arm top cone
  // m_controller.Y().WhileTrue(SetArmPosition(&m_arm, -56000).ToPtr());

  // //middle spot
  // m_controller.X().WhileTrue(SetArmPosition(&m_arm, -64000).ToPtr());

  // //arm down stored
  // m_controller.B().WhileTrue(SetArmPosition(&m_arm, 200).ToPtr());
  // //unlocks the arm from brake mode
  m_controller.Start().WhileTrue(frc2::InstantCommand([this] {m_arm.freeArm();}).ToPtr());


  //
  // ---- NEW CODRIVER BINDINGS
  //


  //Arm telescoping control.
  m_controller.A().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());
  m_controller.Y().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());

  m_controller.X().WhileTrue(SnapTo(&m_arm, &m_controller).ToPtr()); // Snap to the closest known position

  frc2::Trigger([this]{
    return m_sense.Get();
  }).OnTrue(frc2::InstantCommand([this]{m_test.Set(true);}).ToPtr());

  frc2::Trigger([this]{
    return !(m_sense.Get());
  }).OnTrue(frc2::InstantCommand([this]{m_test.Set(false);}).ToPtr());



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::CommandPtr{nullptr};
}
