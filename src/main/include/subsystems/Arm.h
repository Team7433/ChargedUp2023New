// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

#include <frc/smartdashboard/SmartDashboard.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();
  void setArm(double pcgo){m_armMotorOne->Set(ControlMode::PercentOutput, pcgo);}

  void setPosition(double pos){m_armMotorOne->Set(ControlMode::Position, pos);}

  double getPosition(){return m_armMotorOne->GetSelectedSensorPosition();}

  void setAngle();
  double getAngle();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX * m_armMotorOne = new WPI_TalonFX{5};
  WPI_TalonFX * m_armMotorTwo = new WPI_TalonFX{10};

  

  double minArmPos;
  
};
