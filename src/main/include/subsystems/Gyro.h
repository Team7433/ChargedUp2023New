// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>

#include <AHRS.h>
#include <ctre/Phoenix.h>

#include <units/angle.h>

class Gyro : public frc2::SubsystemBase {
 public:
  Gyro();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void Periodic();
  void Reset();
  units::degree_t GetYaw();
  units::degree_t GetPitch();
  units::degree_t GetRoll();
  units::degree_t GetClosestError(units::degree_t target);
  // ctre::phoenix::sensors::PigeonIMU* ReturnGyro() {return m_gyro;}
  AHRS* ReturnGyro() {return m_gyro;}

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  AHRS* m_gyro;
  double m_gyroOffset{0.0};

  // ctre::phoenix::sensors::PigeonIMU* m_gyro = new ctre::phoenix::sensors::PigeonIMU{50};
};