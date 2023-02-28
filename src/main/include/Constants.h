// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 1;

}  // namespace OperatorConstants

namespace DriveTrainConstants {

    //offset of mag encoders to starting position
    constexpr double kEncoderOffsetBR = 225.3;
    constexpr double kEncoderOffsetBL = 66.7;
    constexpr double kEncoderOffsetFR = 353.3;
    constexpr double kEncoderOffsetFL = 177.81;

    //2021 bot encoder values
    // const double kEncoderOffsetFL{351.562};
    // const double kEncoderOffsetFR{248.203};
    // const double kEncoderOffsetBL{4.482};
    // const double kEncoderOffsetBR{323.438};

}

namespace SwerveDriveConstants {
    static constexpr double kJoystickRotateDeadZone{0.25};
    static constexpr double kJoystickStrafeDeadZone{0.15};
    static constexpr double kJoystickForwardDeadZone{0.15};

    static constexpr double kStrafeMultiplier{1.85};
    static constexpr double kForwardMultiplier{1.85};
    static constexpr double kRotateMultiplier{1.25};
}

namespace JoystickDriveConstants {
    constexpr double kJoyStickDampen = 0.5;
}

namespace ArmConstants {
    constexpr unsigned int kArmMotorOne = 5;
    constexpr unsigned int kArmMotorTwo = 10;

    constexpr unsigned int kLimitSwitchID = 0; // Configure this
    constexpr unsigned int kArmEncoderRange = 20000; // Configure this
}