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
  //set percentage output
  void setPercentageOutput(double output);
  
  //set motion magic target
  void setMotionMagic(double position);

  //get the current active target of the motion magic
  double getActiveTrajectoryPosition();

  //get the target position of the motion magic
  double getMotionMagicTargetPosition();

  //get the current encoder count position
  double getPosition();
  
  //get the target pos of the arm motors in encoder counts
  double getTargetPos();

  //disable arm braking mode and set to percentage output mode
  void freeArm();

  //set motors to brake mode
  void enableBrakeMode();


  //Pneumatic control functions
  //Set the state of extension on the arm
  void extendArm(frc::DoubleSolenoid::Value val); 
  //set the grabber state
  void setClaw(frc::DoubleSolenoid::Value state){clawSolenoid.Set(state);}


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double m_motionMagicTarget{0.0};

  WPI_TalonFX * m_armMotorOne = new WPI_TalonFX{kArmMotorOne};
  WPI_TalonFX * m_armMotorTwo = new WPI_TalonFX{kArmMotorTwo};


  frc::DoubleSolenoid clawSolenoid{0, frc::PneumaticsModuleType::CTREPCM, 4, 0};
  frc::DoubleSolenoid extensionSolenoid{frc::PneumaticsModuleType::CTREPCM, 5, 1};

  
};
