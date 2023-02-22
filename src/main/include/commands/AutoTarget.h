// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Vision.h"
#include "subsystems/Gyro.h"
#include "subsystems/SwerveDriveTrain.h"

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "Constants.h"

#include <cmath>

#include <units/math.h>
#include <units/angle.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoTarget
    : public frc2::CommandHelper<frc2::CommandBase, AutoTarget> {
 public:
  AutoTarget(SwerveDrivetrain*, Gyro*, Vision*, frc::Joystick*, frc::XboxController*);

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
  frc::Joystick* m_joystick;
  frc::XboxController* m_controller;

  std::map<std::string, double> kPID{{"kP",0.05}, {"kI", 0.0001}, {"kD", 0.0}, {"kS", 0.01}};
  double static constexpr m_maxAccumulator{100.0};
  double static constexpr m_izone{4.0};
  double m_accumulator;

  units::degree_t m_error{0_deg};
  units::degree_t m_gyroTarget{0_deg};


  bool m_done{false};
};
