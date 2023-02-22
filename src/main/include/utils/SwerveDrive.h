#pragma once

#include "SwerveModule.h"

#include "utils/Vector2D.h"

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <map>
#include <string>
#include <math.h>


namespace iona {

    struct coordinate {
        units::meter_t x_pos;
        units::meter_t y_pos;
    };

    class SwerveDrive {
     public:
        SwerveDrive(SwerveModule* TopLeftModule, SwerveModule* TopRightModule, SwerveModule* BottomLeftModule, SwerveModule* BottomRightModule, units::meter_t trackWidth, units::meter_t wheelBase);

        //direction is given by Forward and strafe values
        void Drive(double FWD, double STR, double rotationValue, units::radian_t gyroAngle);

        //drive with a given vector for move direction
        void Drive(Vector2D MoveDirection, double rotationValue, units::radian_t gyroAngle);
        // Display Data on all swerve modules and on swerve drive
        void DisplayData() const;

        void updateOdometry(units::radians_per_second_t changeInRotation);
        
     private:
        //perform pythag to find the hypotenuse length
        double pythagFindHypot(double x, double y) {return sqrt( pow(x, 2) + pow(y, 2) );};
        //applies the output vectors to the motors
        void UpdateMotorValues();
        //deletes the old output vectors stored
        void deleteOldOutputVecs();


        //pointers to the swerve modules
        SwerveModule* m_moduleFL;
        SwerveModule* m_moduleFR;
        SwerveModule* m_moduleBL;
        SwerveModule* m_moduleBR;

        //physical measurements of where the modules are located on the drive base
        const units::meter_t m_trackWidth;
        const units::meter_t m_wheelBase;
        const units::meter_t m_radius{units::meter_t(pythagFindHypot(m_trackWidth.to<double>()/2, m_wheelBase.to<double>()/2))};

        //stored vector that gets applies to the swerve modules
        std::map<std::string, Vector2D*> outputVector{{"FrontLeft", nullptr}, {"FrontRight", nullptr}, {"BackLeft", nullptr}, {"BackRight", nullptr}};

        //If the swerve modules are set to velocity driving mode
        bool m_velocityDrive{false};

        //coordinate for odometry
        coordinate m_currentPosition{.x_pos=0_m, .y_pos=0_m};

        units::radian_t m_previousAngle{0_rad};
       

    };
    
    
}
