// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include "Constants.h"
#include <iostream>
#include <frc/DoubleSolenoid.h>

using namespace ArmConstants;

#include <frc/smartdashboard/SmartDashboard.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();
  void setArm(double pcgo){m_armMotorOne->Set(ControlMode::PercentOutput, pcgo);}

  void setPosition(double pos){m_armMotorOne->Set(ControlMode::Position, pos); std::cout << "armsetting" << std::endl;}
  double getPosition(){return m_armMotorOne->GetSelectedSensorPosition();}
  double getTargetPos() {return m_armMotorOne->GetClosedLoopTarget();}
  void extendArm(frc::DoubleSolenoid::Value val){extensionSolenoid.Set(val);}

  void setAngle(double angle){} // Still gotta think about foolproof logic
  // double getAngle(){}

  void calibrateArm(){
    if (! m_limitSwitch.Get()){
      m_armMotorOne->SetSelectedSensorPosition(kArmEncoderRange/2); // Setting arm to vertical position
  }}

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX * m_armMotorOne = new WPI_TalonFX{kArmMotorOne};
  WPI_TalonFX * m_armMotorTwo = new WPI_TalonFX{kArmMotorTwo};

  frc::DigitalInput m_limitSwitch{kLimitSwitchID};
  frc::DoubleSolenoid extensionSolenoid{frc::PneumaticsModuleType::CTREPCM, 5, 1};

  
};
