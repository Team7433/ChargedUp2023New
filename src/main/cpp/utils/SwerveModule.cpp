#include "utils/SwerveModule.h"



using namespace iona;

SwerveModule::SwerveModule(int driveMotorID, int pivotMotorID, int CANCoderID, double absEncoderOffset, std::string moduleName, moduleSetup setupInfo) : m_driveMotorID{driveMotorID}, m_pivotMotorID{pivotMotorID}, m_CANCoderID{CANCoderID}, kAbsEncoderOffset{absEncoderOffset}, m_moduleName{moduleName}, k_setupInfo{setupInfo} {

    //Config Factory Defaults
    m_driveMotor->ConfigFactoryDefault();
    m_pivotMotor->ConfigFactoryDefault();
    m_absEncoder->ConfigFactoryDefault();

    //set motors to Brake mode
    m_driveMotor->SetNeutralMode(motorcontrol::Brake);
    m_pivotMotor->SetNeutralMode(motorcontrol::Brake);

    //init encoder on angle motor
    m_pivotMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, kslotIndex, kTimeoutMs);

    //config PID for angle motor
    m_pivotMotor->Config_kP(kslotIndex, m_PID_P["P"], kTimeoutMs);
    m_pivotMotor->Config_kI(kslotIndex, m_PID_P["I"], kTimeoutMs);
    m_pivotMotor->Config_kD(kslotIndex, m_PID_P["D"], kTimeoutMs);

    //invert encoders
    m_pivotMotor->SetSensorPhase(k_setupInfo.kInvertPivotEncoder);

    //config PID for drive motors
    m_driveMotor->Config_kP(kslotIndex, m_PID_D["P"], kTimeoutMs);
    m_driveMotor->Config_kI(kslotIndex, m_PID_D["I"], kTimeoutMs);
    m_driveMotor->Config_kD(kslotIndex, m_PID_D["D"], kTimeoutMs);
    m_driveMotor->Config_kF(kslotIndex, m_PID_D["F"], kTimeoutMs);

    //setup absolute encoder
    m_absEncoder->ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);

    //set up falcon encoders to the right values from the absolute encoders
    resetEncoderPosition();

}


void SwerveModule::resetEncoderPosition() {

    //find the new encoder position
    double newEncoderPosition = (m_absEncoder->GetAbsolutePosition() - kAbsEncoderOffset) * k_setupInfo.kEncoderPerDegree;
    
    //set the falcon motor encoder position 
    m_pivotMotor->SetSelectedSensorPosition(newEncoderPosition, kslotIndex, kTimeoutMs);
}

void SwerveModule::Set(units::meters_per_second_t velocity) {
    //encoder velocity is in encoder counts per 100ms
    double encoderVelocity = (velocity.to<double>() * k_setupInfo.kencoderPerM) / 10; 
    m_driveMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, encoderVelocity); // Set the velocity
}

void SwerveModule::Set(double output) {

    //only set output if its output is greater than the deadzone to prevent jitter
    if(output >= k_setupInfo.kOutputDeadzone) {
        m_driveMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output*m_outputInversion);
    } else {
        m_driveMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }
}

units::radian_t SwerveModule::outputInversion(units::radian_t error) {
    //Absolute value of the error
    double ABSerror = units::math::fabs(error).to<double>();
        
    //if the error is more that 90 Deg and less than 270 Deg, meaning it is the other side then flip the wheel turn direction
    if(ABSerror > M_PI/2 && ABSerror < 3*M_PI/2) {
        m_outputInversion = -1; // flip the drive direction
        
        units::radian_t newError = units::radian_t(error.to<double>() + M_PI); // flip the error to the other side
        newError = units::radian_t(remainder(newError.to<double>(), 2*M_PI)); // map the values back to 0 to 2PI
        
        return newError; 
    }
    m_outputInversion = 1; // set the drive direction to be normal
    return error;
}

units::radian_t SwerveModule::closestPathError(units::radian_t error) {

    if (units::math::fabs(error) <= 90_deg) return error; // if the error is smaller than 90 deg do nothing

    double m_currentDirection = units::math::fabs(error)/error; // the sign of the error -1 or 1 
    
    units::radian_t newError = (360_deg - units::math::fabs(error)); // find the error required to turn in the other direction
    
    newError = newError * -m_currentDirection; //make the sign of the new error to the opposite of the old error

    return newError; 
}

void SwerveModule::Set(units::radian_t heading) { 
    //Only run if the drive output motor has met the output deadzone to stop jitter   
    if(fabs(m_driveMotor->GetMotorOutputPercent()) <= k_setupInfo.kOutputDeadzone) return;

    //checks if there has been encoder drift/encoder init error from motor
    // encoderDriftCheck();

    //Calculate the error from current to the target heading
    units::radian_t error = heading - units::degree_t(remainder(m_pivotMotor->GetSelectedSensorPosition()/ k_setupInfo.kEncoderPerDegree , 360));
    
    //perform output inversion on error
    error = outputInversion(error);

    //perform closest path logic on error
    error = closestPathError(error);

    //set the target encoder count as an offset of the current position added with the error
    double targetEncoderCount = m_pivotMotor->GetSelectedSensorPosition() + units::degree_t(error).to<double>()*k_setupInfo.kEncoderPerDegree;

    m_pivotMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetEncoderCount);

}

void SwerveModule::encoderDriftCheck() {
    //if the difference between the abs encoder and motor encoder is greater than 5 degrees then reset motor encoder position
    if(fabs((m_absEncoder->GetAbsolutePosition()-kAbsEncoderOffset) - (m_pivotMotor->GetSelectedSensorPosition()/k_setupInfo.kEncoderPerDegree)) > 5) {
        resetEncoderPosition();
    }

}



moduleInfo SwerveModule::getInfo() {
    moduleInfo info;
    //fill data on module name
    info.moduleName = m_moduleName;
    
    //fill data into struct on the motor percentage outputs
    info.driveMotorOutput = m_driveMotor->GetMotorOutputPercent();
    info.pivotMotorOutput = m_pivotMotor->GetMotorOutputPercent();

    //fill data on module velocity
    info.driveSpeed = units::meters_per_second_t(m_driveMotor->GetSelectedSensorVelocity()/(k_setupInfo.kencoderPerM*10));
    
    //fill data  on the heading angle from both encoder, absolute and falcon motor
    info.headingAngle_a = units::degree_t(m_absEncoder->GetAbsolutePosition());
    
    if(m_pivotMotor->GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::Position) {
    //fill data on target encoder count
        info.headingAngle_e = units::degree_t(remainder(m_pivotMotor->GetSelectedSensorPosition()/k_setupInfo.kEncoderPerDegree, 360));
        info.targetAngleEncoderC = m_pivotMotor->GetClosedLoopTarget();
    } else {
        info.targetAngleEncoderC = 0;
        info.headingAngle_e = 0_deg;
    }
    //if drive motor is in velocity control then add info about its status
    if(m_driveMotor->GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::Velocity) {
        info.targetDriveEncoderVel = m_driveMotor->GetClosedLoopTarget();
    } else {
        info.targetDriveEncoderVel = 0;
    }

    return info;
}



void SwerveModule::displayModuleData() {
    moduleInfo info = getInfo();

    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/moduleDriveVelocity", info.driveSpeed.to<double>());
    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/moduleDriveOutput", info.driveMotorOutput);
    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/modulePivotOutput", info.pivotMotorOutput);


    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/MotorEncoderHeadingAngle", info.headingAngle_e.to<double>());
    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/AbsoluteHeadingAngle", info.headingAngle_a.to<double>());
    

    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/TargetAngleEncoderCount", info.targetAngleEncoderC);
    frc::SmartDashboard::PutNumber("SwerveDrive/" + m_moduleName + "/TargetDriveEncoderVel", info.targetDriveEncoderVel);

}

Vector2D SwerveModule::getDirection() {

    moduleInfo tempInfo = getInfo();

    Vector2D directionVector{units::radian_t(tempInfo.headingAngle_e.to<double>()*(M_PI)/(180)), units::meter_t(tempInfo.driveSpeed.to<double>())};
    return directionVector;

}





