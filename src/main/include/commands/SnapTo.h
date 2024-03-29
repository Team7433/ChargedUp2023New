// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Arm.h"
#include "frc2/command/button/CommandXboxController.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SnapTo
    : public frc2::CommandHelper<frc2::CommandBase, SnapTo> {
 public:
  SnapTo(Arm * arm, frc2::CommandXboxController* controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  bool isControllerActive();

 private:

    Arm * m_arm;
    frc2::CommandXboxController * m_controller;
    double m_target;


    std::map<std::string, int> armPositions = {
        {"Retracted", 0},
        {"TopGrid", -54000},
        {"MidGrid", -62000},
        {"Collection", -77900}
    };

};






