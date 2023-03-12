// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrivetrain.h"
#include "subsystems/Gyro.h"
#include <units/angle.h>
#include "frc/Timer.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BotBalance
    : public frc2::CommandHelper<frc2::CommandBase, BotBalance> {
 public:
  BotBalance(SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;



  private:
  units::degree_t getRotationError();
  units::radian_t closestPathError(units::radian_t error);

  // TODO threshold
  units::degree_t m_threshold = 10_deg;
  
  Gyro * m_gyro;
  SwerveDrivetrain * m_swerveDrive;
  double pi = 3.14159265358979323846264;

  frc::Timer m_timer;

  bool m_done{false};

};
