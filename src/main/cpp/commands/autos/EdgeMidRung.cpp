// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/EdgeMidRung.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
EdgeMidRung::EdgeMidRung(Arm * m_arm, SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  AddCommands(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this, m_swerveDrive] { m_swerveDrive->ResetOdometry();}), // Reset Odometry
      SetArmPosition(m_arm, -52000), // Set to above mid cone position  
      SetArmPosition(m_arm, -56000), // Place cone down into mid
      frc2::InstantCommand([this, m_arm]{ m_arm->setClaw(frc::DoubleSolenoid::kForward);}), // Release cone
      Wait(0.1_s), // Wait for arm to retract

      frc2::ParallelCommandGroup( // Set arm vertical and move simultaneously
      SetArmPosition(m_arm, -48000), // Arm up
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 3_m, .y_pos = 0_m}, 180_deg, MoveToConfig{.maxVelocity = 2.5_mps, .Acceleration = 2.5_mps_sq}))
      ));
}
