// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc2/command/InstantCommand.h>
#include "subsystems/SwerveDrivetrain.h"
#include "commands/SetArmPosition.h"
#include <frc/DoubleSolenoid.h>
#include "subsystems/Arm.h"
#include <frc2/command/ParallelCommandGroup.h>
#include "commands/MoveTo.h"
#include "commands/Wait.h"
#include "commands/BotBalance.h"

class ChargeStationMidRung
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ChargeStationMidRung> {
 public:
  ChargeStationMidRung(Arm * m_arm, SwerveDrivetrain * m_swerveDrive, Gyro * m_gyro);
};
