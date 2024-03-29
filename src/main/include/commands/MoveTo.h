// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrivetrain.h"

#include <frc/Timer.h>
#include <math.h>

#include "units/angular_velocity.h"
#include "units/angular_acceleration.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

struct MoveToConfig {

  units::meters_per_second_t maxVelocity;
  units::meters_per_second_squared_t Acceleration;

};


class MoveTo
    : public frc2::CommandHelper<frc2::CommandBase, MoveTo> {
 public:
  MoveTo(SwerveDrivetrain* SwerveDrive, Gyro* gyro, iona::coordinate targetCoordinate, units::degree_t faceFirection);
  MoveTo(SwerveDrivetrain* SwerveDrive, Gyro* gyro, iona::coordinate targetCoordinate, units::degree_t faceFirection, MoveToConfig);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:

   units::radian_t closestPathError(units::radian_t error);


   units::radian_t getMoveDirection();
   units::meter_t getDistanceLeft();
   units::second_t getDeltaTime();
   units::degree_t getRotationError();



  iona::coordinate m_targetCoordinate;
  SwerveDrivetrain* m_swerveDrive;
  Gyro* m_gyro;

  units::meters_per_second_t m_maxVelocity{4_mps};
  units::meters_per_second_squared_t m_maxAcceleration{3_mps_sq};


  units::meters_per_second_t m_newVelocity{0.0_mps};
  units::meters_per_second_t m_currentVelocity{0.0_mps};


  units::meters_per_second_squared_t m_angleAcceleration{0.5_mps_sq};
  units::meters_per_second_t m_maxAngleVelocity{1.5_mps};

  units::meters_per_second_t m_newAngleVelocity{0.0_mps};
  units::meters_per_second_t m_currentAngleVelocity{0.0_mps};

  double m_rotateKP{0.015};
  double m_rotateKI{0.001};

  double m_accumulator{0.0};

  units::degree_t m_targetFaceDirection{0_deg};

  frc::Timer m_timer;

};
