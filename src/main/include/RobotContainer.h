// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/XboxController.h>

#include "subsystems/Arm.h"
#include "subsystems/Claw.h"
#include "subsystems/Compressor.h"
#include "subsystems/Gyro.h"
#include "subsystems/SwerveDrivetrain.h"
#include "subsystems/Vision.h"


#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "commands/DriveWithJoystick.h"
#include "commands/TurnToTarget.h"
#include "commands/JoystickArmControl.h"
#include "commands/HoldArm.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  void setCompressorWrapper(bool state) {
    m_compressor.setCompressorOn(state);
  }

  

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{0};
  frc::Joystick m_joystick{1};
  frc::XboxController m_controller{0};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Claw m_claw;
  Arm m_arm;
  Compressor m_compressor;
  SwerveDrivetrain m_swerveDrive;
  Gyro m_gyro;
  Vision m_vision;
  
  

  void ConfigureBindings();
};
