// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/LeaveCommunityLeft.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
LeaveCommunityLeft::LeaveCommunityLeft(SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  AddCommands(
    // Wait(5_s);
    MoveTo(m_swerveDrive, m_gyro, iona::coordinate{.x_pos = 2_m, .y_pos = 0.0_m}, 180_deg, MoveToConfig{ .maxVelocity = 1_mps, .Acceleration = 0.65_mps_sq})
  );
}
