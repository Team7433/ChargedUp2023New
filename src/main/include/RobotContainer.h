// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc/XboxController.h>
#include <frc/DigitalOutput.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/Arm.h"
#include "subsystems/Compressor.h"
#include "subsystems/Gyro.h"
#include "subsystems/SwerveDrivetrain.h"
#include "subsystems/Vision.h"


#include "Constants.h"

#include "commands/DriveWithJoystick.h"
#include "commands/TurnToTarget.h"
#include "commands/JoystickArmControl.h"
#include "commands/AutoTarget.h"
#include "commands/MoveTo.h"
#include "commands/Wait.h"
#include "commands/SnapTo.h"
#include "commands/BotBalance.h"
#include "commands/SetArmPosition.h"

#include "commands/autos/TwoItemAutoRight.h"
#include "commands/autos/OneItemMid.h"
#include "commands/autos/TwoItemAutoRightTwo.h"
#include "commands/autos/LeaveCommunityLeft.h"
#include "commands/autos/OneItemEdge.h"
#include "commands/autos/ChargeStationMidRung.h"
#include "commands/autos/EdgeMidRung.h"
#include "commands/autos/DeliverOne.h"

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

  void armPercentageOutput(double output) {
    m_arm.setPercentageOutput(output);
  }

  void setCompressorWrapper(bool state) {
    m_compressor.setCompressorOn(state);
  }

  

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_controller{1};
  frc::Joystick m_driverStick{0};

  // The robot's subsystems are defined here...
  Arm m_arm;
  Compressor m_compressor;
  SwerveDrivetrain m_swerveDrive;
  Gyro m_gyro;
  Vision m_vision;

  frc::SendableChooser<int> autoSelecter;


  
  

  void ConfigureBindings();
};
