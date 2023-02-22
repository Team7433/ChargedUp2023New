// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Vision.h"
#include "subsystems/Gyro.h"
#include "subsystems/SwerveDrivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurnToTarget
    : public frc2::CommandHelper<frc2::CommandBase, TurnToTarget> {
 public:
  TurnToTarget(SwerveDrivetrain* swerveDriveTrain, Gyro* gyro, Vision* vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  units::degree_t getCLosestError(units::degree_t targetGyroAngle); //finds closest path (error) to target
  units::degree_t getRemappedGyroAngle(units::degree_t targetGyroAngle); // maps like 190 degrees to -170 Degrees


  private:
  Gyro* m_gyro;
  Vision* m_vision;
  SwerveDrivetrain* m_swerveDrive;


  std::map<std::string, double> kPID{{"kP",0.1
  }, {"kI", 0.0001}, {"kD", 0.0}, {"kS", 0.01}};
  double static constexpr m_maxAccumulator{100.0};
  double static constexpr m_izone{4.0};
  double m_accumulator;

  units::degree_t m_error{0_deg};
  units::degree_t m_gyroTarget{0_deg};
};
