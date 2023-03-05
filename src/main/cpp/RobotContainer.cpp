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
  // m_arm.SetDefaultCommand(JoystickArmControl(&m_arm, &m_controller));

  // SNAP TO POSITION ARM CONTROL
  m_arm.SetDefaultCommand(SnapTo(&m_arm, &m_controller, false));

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
  // frc2::Trigger([this]{ return m_driverStick.GetRawButton(4); }).OnTrue(
  //   frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([this] {m_swerveDrive.ResetOdometry();}),
  //     SetArmPosition(&m_arm, -60000),
  //     frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);}),
  //     MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.8_m, .y_pos = 0.0_m}, 0_deg),
  //     MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.8_m, .y_pos = 0.10_m}, 0_deg),
  //     SetArmPosition(&m_arm, -76000),
  //     frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}),
  //     SetArmPosition(&m_arm, -50000),
  //     MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.80_m, .y_pos = 0.0_m}, 180_deg),
  //     MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0.00_m, .y_pos = 0.0_m}, 180_deg),
  //     frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);})


  //   ).ToPtr());

  frc2::Trigger([this]{ return m_driverStick.GetRawButton(6); }).OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] {m_swerveDrive.ResetOdometry();}),
      SetArmPosition(&m_arm, -54000),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);}),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0.0_m, .y_pos = -0.7_m}, 180_deg),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 4.8_m, .y_pos = -0.7_m}, 0_deg),
      SetArmPosition(&m_arm, -78000),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kReverse);}),
      SetArmPosition(&m_arm, -55000),
      MoveTo(&m_swerveDrive, &m_gyro, iona::coordinate{.x_pos = 0.0_m, .y_pos = -0.7_m}, 180_deg),
      frc2::InstantCommand([this] {m_arm.setClaw(frc::DoubleSolenoid::kForward);})


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
  // frc2::Trigger([this] {return (m_controller.GetRightTriggerAxis() > 0.6);}).OnTrue(
  //   frc2::InstantCommand([this] { std::cout << "ArmOut!\n"; m_arm.extendArm(frc::DoubleSolenoid::kForward);}).ToPtr()

  // );


  // frc2::Trigger([this] {return m_controller.GetLeftTriggerAxis() > 0.6;}).OnTrue(
  //   frc2::InstantCommand([this] {std::cout << "Arm In!\n"; m_arm.extendArm(frc::DoubleSolenoid::kReverse);}).ToPtr()

  // );


  // //Arm move to collect cone position
  // m_controller.A().WhileTrue(SetArmPosition(&m_arm, -83000).ToPtr());

  // //Arm top cone
  // m_controller.Y().WhileTrue(SetArmPosition(&m_arm, -56000).ToPtr());

  // //middle spot
  // m_controller.X().WhileTrue(SetArmPosition(&m_arm, -64000).ToPtr());

  // //arm down stored
  // m_controller.B().WhileTrue(SetArmPosition(&m_arm, 200).ToPtr());
  // //unlocks the arm from brake mode
  // m_controller.Start().WhileTrue(frc2::InstantCommand([this] {m_arm.freeArm();}).ToPtr());


  //
  // ---- NEW CODRIVER BINDINGS
  //


  //Arm telescoping control.
  m_controller.A().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kForward);}).ToPtr());
  m_controller.Y().WhileTrue(frc2::InstantCommand([this] {m_arm.extendArm(frc::DoubleSolenoid::Value::kReverse);}).ToPtr());

  m_controller.X().WhileTrue(SnapTo(&m_arm, &m_controller, true).ToPtr()); // Snap to the closest known position;

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::CommandPtr{nullptr};
}
