// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "utils/SwerveModule.h"
#include "utils/Vector2D.h"
#include "utils/SwerveDrive.h"
#include "subsystems/Gyro.h"

#include <ctre/Phoenix.h>
// #include <frc/geometry/Translation2d.h>
// #include <frc/kinematics/SwerveDriveOdometry.h>
// #include <frc/kinematics/SwerveDriveKinematics.h>
#include <map>
#include <AHRS.h>
#include <frc/SPI.h>

#include <Constants.h>
using namespace DriveTrainConstants;

class SwerveDrivetrain : public frc2::SubsystemBase {
 public:
  SwerveDrivetrain(Gyro* gyro);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Drive(units::radian_t direction, units::meter_t magnitude, double rotation);

  void ResetOdometry() {m_swerveDrive->resetOdometryCoordinate();}

  iona::coordinate getOdometryCoordinate() const {return m_swerveDrive->getOdometryCoordinate();}

  // void SetupDriveTo(units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t acceleration, units::radians_per_second_t maxRotateSpeed, units::radians_per_second_squared_t rotAcceleration);
  // void SetDriveTarget(units::meter_t targetX, units::meter_t targetY, units::radian_t targetZ);

  // void ResetGyro() {gyro.Reset();}

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Swerve module settings
  //old wheel circumference 0.2892_m
  iona::moduleSetup swerveModuleSettings{.kEncoderPerDegree = (2048*12.8)/360.0, .kOutputDeadzone = 0.05, .kInvertPivotEncoder = true, .kWheelCircumference = 0.31919_m, .kencoderPerM = (2048.0*(6.75))/0.31919};
  

  units::meter_t const kWheelBase = 546.1_mm;
  units::meter_t const kTrackWidth = 596.9_mm;

  units::meter_t halveWheelBase = kWheelBase/2;
  units::meter_t  halveTrackWidth = kTrackWidth/2;

  iona::SwerveModule* m_swerveModuleBR = new iona::SwerveModule(41, 42, 43, kEncoderOffsetBR, "BackRight", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleBL = new iona::SwerveModule(31, 32, 33, kEncoderOffsetBL, "BackLeft", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleFR = new iona::SwerveModule(21, 22, 23, kEncoderOffsetFR, "FrontRight", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleFL = new iona::SwerveModule(11, 12, 13, kEncoderOffsetFL, "FrontLeft", swerveModuleSettings);

  iona::SwerveDrive* m_swerveDrive = new iona::SwerveDrive(m_swerveModuleFL, m_swerveModuleFR, m_swerveModuleBL, m_swerveModuleBR, kTrackWidth, kWheelBase);

  PigeonIMU* m_gyro2 = new PigeonIMU{50};
  

  Gyro* m_gyro;

};