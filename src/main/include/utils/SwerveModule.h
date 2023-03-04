#pragma once

#include <ctre/Phoenix.h>

#include <map>
#include <string>

#include <iostream>

#include <units/angle.h>
#include <units/velocity.h>
#include <units/math.h>
#include <units/acceleration.h>
#include <utils/Vector2D.h>


#include <frc/smartdashboard/SmartDashboard.h>


# define M_PI 3.14159265358979323846  /* pi */


namespace iona {

    struct moduleSetup {

        const double kEncoderPerDegree; // encoder counts it takes to rotate the WHEEK by one degreee

        const double kOutputDeadzone; //percentage output required for the swerve to output

        const bool kInvertPivotEncoder; // invert angle motor encoder direction

        const units::meter_t kWheelCircumference; // the circumference of the swerve wheels

        const double kencoderPerM; // amount of encoder turns required to travel 1 meter

    };


    //returns a moduleInfo to get any of the data
    struct moduleInfo {

        std::string moduleName; // name given to the module
        

        units::meters_per_second_t driveSpeed; //speed of the module
        double driveMotorOutput; //percentage output on motor

        units::degree_t headingAngle_e; //Falcon encoder heading angle
        units::degree_t headingAngle_a; //Absolute encoder heading angle

        double targetAngleEncoderC; //target encoder count for angle
        double targetDriveEncoderVel; // target drive encoder velocity

        double targetEnocoderAngle;


        double pivotMotorOutput; // percentage output on pivot motor

    };



    class SwerveModule {
     public:
        SwerveModule(int driveMotorID, int pivotMotorID, int CANCoderID, double absEncoderOffset, std::string moduleName, moduleSetup setupInfo); // class initialiser
        
        void resetEncoderPosition(); //Sets the falcon pivot motor encoder to match with mag absolute encoder        
        
        //Set drive motor velocity
        void Set(units::meters_per_second_t velocity);
        
        //Set the heading of the swerve module
        void Set(units::radian_t heading);
        
        // Set the percentage output of the drive motor
        void Set(double output); // Set the drive percentage output

        // Gets the module info
        moduleInfo getInfo();
     
        //display module info onto smartdashboard
        void displayModuleData();
     
        void setInversion(bool state) {
            m_driveMotor->SetInverted(state);
        }

        Vector2D getDirection();
     
     private:
        
        

        //output inversion logic to flip drive direction if the error is larger than 180
        units::radian_t outputInversion(units::radian_t error);
        int m_outputInversion{1};

        //calculates a new error if a closer error is found eg turning anticlockwise instead of clockwise
        units::radian_t closestPathError(units::radian_t error);





        //CAN ID for motors
        const int m_driveMotorID;
        const int m_pivotMotorID;

        //CAN ID for encoder
        const int m_CANCoderID;

        const double kAbsEncoderOffset; // amount of offset added to the absolute encoder to keep it orientated
        
        std::string m_moduleName; // Name of the module
        
        const double kTimeoutMs{10.0}; //ktimeout for PID
        const int kslotIndex{0}; // default kslotIndex for PID

        const moduleSetup k_setupInfo; //setup info


        //PID values for swerve module
        std::map<std::string, double> m_PID_P{{"P", 0.2}, {"I", 0.0007}, {"D", 0.1}}; // pivot motor PID values
        std::map<std::string, double> m_PID_D{{"P", 0.13}, {"I", 0.0007}, {"D", 0.19}, {"F", (1023.0)/21797}}; // drive motor PID values

        //Falcon Motors
        WPI_TalonFX* m_driveMotor = new WPI_TalonFX{m_driveMotorID};
        WPI_TalonFX* m_pivotMotor = new WPI_TalonFX{m_pivotMotorID};

        //CAN Encoder
        WPI_CANCoder* m_absEncoder = new WPI_CANCoder{m_CANCoderID};

    };

}