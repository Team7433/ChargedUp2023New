// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "utils/SwerveModule.h"
#include "utils/Vector2D.h"
#include "utils/SwerveDrive.h"


#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <map>

#include <Constants.h>
using namespace DriveTrainConstants;

class SwerveDrivetrain : public frc2::SubsystemBase {
 public:
  SwerveDrivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Drive(units::radian_t direction, units::meter_t magnitude, double rotation, units::radian_t gyroAngle);

  // void SetupDriveTo(units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t acceleration, units::radians_per_second_t maxRotateSpeed, units::radians_per_second_squared_t rotAcceleration);
  // void SetDriveTarget(units::meter_t targetX, units::meter_t targetY, units::radian_t targetZ);

  void ResetGyro() {m_gyro->SetFusedHeading(0);};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Swerve module settings
  iona::moduleSetup swerveModuleSettings{.kEncoderPerDegree = (2048*12.8)/360.0, .kOutputDeadzone = 0.05, .kInvertPivotEncoder = true, .kWheelCircumference = 0.2892_m, .kencoderPerM = 1};
  
  units::meter_t const kWheelBase = 548.6_mm;
  units::meter_t const kTrackWidth = 596.9_mm;

  units::meter_t halveWheelBase = kWheelBase/2;
  units::meter_t  halveTrackWidth = kTrackWidth/2;

  iona::SwerveModule* m_swerveModuleBR = new iona::SwerveModule(41, 42, 43, kEncoderOffsetBR, "BackRight", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleBL = new iona::SwerveModule(31, 32, 33, kEncoderOffsetBL, "BackLeft", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleFR = new iona::SwerveModule(21, 22, 23, kEncoderOffsetFR, "FrontRight", swerveModuleSettings);
  iona::SwerveModule* m_swerveModuleFL = new iona::SwerveModule(11, 12, 13, kEncoderOffsetFL, "FrontLeft", swerveModuleSettings);

  iona::SwerveDrive* m_swerveDrive = new iona::SwerveDrive(m_swerveModuleFL, m_swerveModuleFR, m_swerveModuleBL, m_swerveModuleBR, kTrackWidth, kWheelBase);

  std::map<std::string, frc::Translation2d> m_translations{{"TopRight", frc::Translation2d{halveTrackWidth, halveWheelBase}}, {"TopLeft", frc::Translation2d{-halveTrackWidth, halveWheelBase}}, {"BottomLeft", frc::Translation2d{-halveTrackWidth, -halveWheelBase}}, {"BottomRight", frc::Translation2d{halveTrackWidth, -halveWheelBase}}};

  // frc::SwerveDriveKinematics<4> m_kinematics{m_translations["TopLeft"], m_translations["TopRight"], m_translations["BottomLeft"], m_translations["BottomRight"]};

  // frc::SwerveDriveOdometry<4> m_odometry(m_kinematics);


  PigeonIMU* m_gyro = new PigeonIMU{50};

};