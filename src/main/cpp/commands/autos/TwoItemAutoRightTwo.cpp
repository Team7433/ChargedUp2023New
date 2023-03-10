// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/TwoItemAutoRightTwo.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoItemAutoRightTwo::TwoItemAutoRightTwo(Arm * m_arm, SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  
  AddCommands(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this, m_swerveDrive] { m_swerveDrive->ResetOdometry();}), // Reset Odometry
      SetArmPosition(m_arm, -52000), // Set to above high cone position  
      frc2::InstantCommand([this, m_arm]{ m_arm->extendArm(frc::DoubleSolenoid::kForward);}), // Extend arm
      Wait(0.2_s), // Wait for arm to extend
      SetArmPosition(m_arm, -56000), // Place cone down into high
      frc2::InstantCommand([this, m_arm]{ m_arm->setClaw(frc::DoubleSolenoid::kForward);}), // Release cone
      frc2::InstantCommand([this, m_arm]{m_arm->extendArm(frc::DoubleSolenoid::kReverse);}), // Retract arm telescoping
      Wait(0.1_s), // Wait for arm to retract

      frc2::ParallelCommandGroup( // Set arm vertical and move simultaneously
      SetArmPosition(m_arm, -48000), // Arm up
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 0.5_m, .y_pos = 1_m}, 180_deg, MoveToConfig{.maxVelocity = 2.5_mps, .Acceleration = 2.5_mps_sq})),

      // Go near next cube
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 4.5_m, .y_pos = 1_m}, 0_deg),

      frc2::ParallelCommandGroup( // Put arm down and go to cube simultaneously
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 4.75_m, .y_pos = 0.25_m}, 0_deg),
      SetArmPosition(m_arm, -83000)), // Arm to collect

      frc2::InstantCommand([this, m_arm]{ m_arm->setClaw(frc::DoubleSolenoid::kReverse);}), // Collect cube

      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 4.65_m, .y_pos = 0.5_m}, 0_deg,  MoveToConfig{.maxVelocity = 1.5_mps, .Acceleration = 2.5_mps_sq}),
      
      frc2::ParallelCommandGroup(
      SetArmPosition(m_arm, -56000),
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 0.1_m, .y_pos = 0.65_m}, 180_deg)), // Move to grids
      frc2::InstantCommand([this, m_arm]{ m_arm->extendArm(frc::DoubleSolenoid::kForward);}), // Extend arm
      Wait(0.45_s), // Wait for arm to extend
      frc2::InstantCommand([this, m_arm]{ m_arm->setClaw(frc::DoubleSolenoid::kForward);}) // Drop cone


      ));
}
