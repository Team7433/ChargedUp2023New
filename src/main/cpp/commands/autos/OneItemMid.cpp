// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/OneItemMid.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneItemMid::OneItemMid(Arm * m_arm, SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  AddCommands(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this, m_swerveDrive] {m_swerveDrive->ResetOdometry();}),
      SetArmPosition(m_arm, -58000),
      frc2::InstantCommand([this, m_arm] {m_arm->extendArm(frc::DoubleSolenoid::kForward);}),
      Wait(0.45_s),
      frc2::InstantCommand([this, m_arm] {m_arm->setClaw(frc::DoubleSolenoid::kForward);}),
      Wait(0.2_s),
      frc2::InstantCommand([this, m_arm] {m_arm->extendArm(frc::DoubleSolenoid::kReverse);}),
      Wait(1_s),
      frc2::InstantCommand([this, m_arm] {m_arm->setClaw(frc::DoubleSolenoid::kReverse);}),
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(
 
        SetArmPosition(m_arm, -1000),
        frc2::InstantCommand([this, m_arm] {m_arm->freeArm();})

        ),
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 3.1_m, .y_pos = 0.0_m}, 180_deg, MoveToConfig{ .maxVelocity = 1.5_mps, .Acceleration = 2_mps_sq})
      ),
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 4.7_m, .y_pos = 0.0_m}, 180_deg, MoveToConfig{ .maxVelocity = 1_mps, .Acceleration = 1_mps_sq}),
      MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 3.35_m, .y_pos = 0.0_m}, 180_deg, MoveToConfig{ .maxVelocity = 1_mps, .Acceleration = 1_mps_sq}),
      BotBalance(m_swerveDrive, m_gyro)


      


    )
  );
}
