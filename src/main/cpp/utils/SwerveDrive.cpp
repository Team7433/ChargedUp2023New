#include "utils/SwerveDrive.h"

using namespace iona;

SwerveDrive::SwerveDrive(SwerveModule* TopLeftModule, SwerveModule* TopRightModule, SwerveModule* BottomLeftModule, SwerveModule* BottomRightModule, units::meter_t trackWidth, units::meter_t wheelBase) : m_moduleFL{TopLeftModule}, m_moduleFR{TopRightModule} , m_moduleBL{BottomLeftModule}, m_moduleBR{BottomRightModule}, m_trackWidth{trackWidth}, m_wheelBase{wheelBase} {

}

void SwerveDrive::Drive(double FWD, double STR, double rotationValue, units::radian_t gyroAngle) {
    Vector2D direction{units::meter_t(FWD), units::meter_t(STR)};

    Drive(direction, rotationValue, gyroAngle);
}

void SwerveDrive::Drive(Vector2D MoveDirection, double rotationValue, units::radian_t gyroAngle) {

    //applies field centric movement
    MoveDirection.setDirection(MoveDirection.getDirection() - gyroAngle);

    //rotation is flipped as per unit circle clockwise is a negative value
    rotationValue = rotationValue * -1;


    //IMPORTANT LOGIC REFERENCE
    //logic assumes unit circle starts at zero, counter clock wise is positive and 0 is in the direction of travel positively
    //create the vectors that are tangent to the circle that the 4 swerve modules sit on for each swerve module
    
    Vector2D FrontLeftTang{units::math::atan2(m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(rotationValue)};

    Vector2D FrontRightTang{units::math::atan2(-m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(rotationValue)};

    Vector2D BottomLeftTang{units::math::atan2(m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(rotationValue)};

    Vector2D BottomRightTang{units::math::atan2(-m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(rotationValue)};


    //set the output vector to be the sum of the tangent vector and the over move direction vector
    deleteOldOutputVecs(); // cleans up the old vectors stored
    outputVector["FrontLeft"] = (FrontLeftTang + MoveDirection);
    outputVector["FrontRight"] =  (FrontRightTang + MoveDirection);
    outputVector["BackLeft"] = (BottomLeftTang + MoveDirection);
    outputVector["BackRight"] = (BottomRightTang + MoveDirection);


    //Applies the output vector to the motors
    UpdateMotorValues();
}

void SwerveDrive::UpdateMotorValues() {

    //if velocity drive mode is set to true then treat output vector as a meter per second value
    if(m_velocityDrive) {
        m_moduleFL->Set(outputVector["FrontLeft"]->getMagnitude()/1_s);
        m_moduleFR->Set(outputVector["FrontRight"]->getMagnitude()/1_s);
        m_moduleBL->Set(outputVector["BackLeft"]->getMagnitude()/1_s);
        m_moduleBR->Set(outputVector["BackRight"]->getMagnitude()/1_s);

    } else {
    // //Set drive output from the magnitude of the output vector
        m_moduleFL->Set(outputVector["FrontLeft"]->getMagnitude().to<double>());
        m_moduleFR->Set(outputVector["FrontRight"]->getMagnitude().to<double>());
        m_moduleBL->Set(outputVector["BackLeft"]->getMagnitude().to<double>());
        m_moduleBR->Set(outputVector["BackRight"]->getMagnitude().to<double>());
    }
    //Set angle motors from the direction of the output vector
    m_moduleFL->Set(outputVector["FrontLeft"]->getDirection());
    m_moduleFR->Set(outputVector["FrontRight"]->getDirection());
    m_moduleBL->Set(outputVector["BackLeft"]->getDirection());
    m_moduleBR->Set(outputVector["BackRight"]->getDirection());


}

//remove the old vector objects in the map from memory
void SwerveDrive::deleteOldOutputVecs() {
    delete outputVector["FrontLeft"];
    delete outputVector["FrontRight"];
    delete outputVector["BackLeft"];
    delete outputVector["BackRight"];

}

//display data on swerve modules
void SwerveDrive::DisplayData() const {
    m_moduleFR->displayModuleData();
    m_moduleFL->displayModuleData();
    m_moduleBL->displayModuleData();
    m_moduleBR->displayModuleData();

    frc::SmartDashboard::PutNumber("xCoordinate", m_currentPosition.x_pos.to<double>());
    frc::SmartDashboard::PutNumber("yCoordinate", m_currentPosition.y_pos.to<double>());

}

void SwerveDrive::updateOdometry(units::radians_per_second_t angularSpeed) {

    // units::radian_t deltaAngle{currentGyroAngle - m_previousAngle};

    // m_previousAngle = currentGyroAngle;

    // units::radians_per_second_t angularSpeed = deltaAngle/20_ms;

    // Vector2D* summedVector = m_moduleBL->getDirection() + m_moduleBR->getDirection();
    // summedVector = *summedVector + m_moduleFL->getDirection();
    // summedVector = *summedVector + m_moduleFR->getDirection();


    Vector2D FrontRightTang{units::math::atan2(-m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(angularSpeed.to<double>()*m_radius.to<double>())};

    Vector2D* summedVector = m_moduleFR->getDirection() - FrontRightTang;

    // summedVector->setDirection(summedVector->getDirection() - currentGyroAngle);

    std::cout << "Direction: " << summedVector->getDirection().to<double>()*180/M_PI << " Speed: " << summedVector->getMagnitude().to<double>() << std::endl;

    m_currentPosition.x_pos = units::meter_t(summedVector->getX().to<double>()*0.02) + m_currentPosition.x_pos;
    m_currentPosition.y_pos = units::meter_t(summedVector->getY().to<double>()*0.02) + m_currentPosition.y_pos;

}


