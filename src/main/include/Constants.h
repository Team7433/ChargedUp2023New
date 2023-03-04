// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <map>
#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */


namespace DriveTrainConstants {

    //offset of mag encoders to starting position
    constexpr double kEncoderOffsetBR = 225.3;
    constexpr double kEncoderOffsetBL = 66.7;
    constexpr double kEncoderOffsetFR = 353.3;
    constexpr double kEncoderOffsetFL = 177.81;

    //2022 bot encoder values
    // const double kEncoderOffsetFL{90.0};
    // const double kEncoderOffsetFR{84.5};
    // const double kEncoderOffsetBL{54.4};
    // const double kEncoderOffsetBR{339.1};



}


namespace JoystickDriveConstants {
    constexpr double kJoyStickDampen = 0.5;
}

namespace ArmConstants {
    constexpr unsigned int kArmMotorOne = 5;
    constexpr unsigned int kArmMotorTwo = 10;

    std::map<std::string, int> armPositions = {
        {"Retracted", 00},
        {"TopGrid", -56000},
        {"MidGrid", -64000},
        {"Collection", -83000}
    };

}

namespace VisionConstants {
    constexpr double kPixelCount = 10000; // Configure this.
}