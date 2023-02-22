// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>
#include <frc/SPI.h>
#include <iostream>

class Gyro : public frc2::SubsystemBase {
 public:
  Gyro();

  double GetRotation() {return -gyro.GetYaw();}
  double GetRotationChange() {return -gyro.GetRate();}
  void Reset(){gyro.Reset(); std::cout << "gyro reset!!!" << std::endl;}
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  AHRS gyro{frc::SPI::Port::kMXP};
};
