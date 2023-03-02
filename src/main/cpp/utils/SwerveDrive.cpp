#include "utils/SwerveDrive.h"

using namespace iona;

SwerveDrive::SwerveDrive(SwerveModule* TopLeftModule, SwerveModule* TopRightModule, SwerveModule* BottomLeftModule, SwerveModule* BottomRightModule, units::meter_t trackWidth, units::meter_t wheelBase) : m_moduleFL{TopLeftModule}, m_moduleFR{TopRightModule} , m_moduleBL{BottomLeftModule}, m_moduleBR{BottomRightModule}, m_trackWidth{trackWidth}, m_wheelBase{wheelBase} {

    

}

tangentVectors SwerveDrive::getTangentVectors(units::meter_t magnitude) {

    //IMPORTANT LOGIC REFERENCE
    //logic assumes unit circle starts at zero, counter clock wise is positive and 0 is in the direction of travel positively
    //create the vectors that are tangent to the circle that the 4 swerve modules sit on for each swerve module
    
    Vector2D FrontLeftTang{units::math::atan2(m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(magnitude)};

    Vector2D FrontRightTang{units::math::atan2(-m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(magnitude)};

    Vector2D BottomLeftTang{units::math::atan2(m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(magnitude)};

    Vector2D BottomRightTang{units::math::atan2(-m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(magnitude)};

    //create tangetVectors struct to be prepared to be returned
    tangentVectors vectors{.FrontLeft = FrontLeftTang, .FrontRight = FrontRightTang, .BackLeft = BottomLeftTang, .BackRight = BottomRightTang};

    return vectors;
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

    frc::SmartDashboard::PutNumber("SwerveDrive/xCoordinate", m_currentPosition.x_pos.to<double>());
    frc::SmartDashboard::PutNumber("SwerveDrive/yCoordinate", m_currentPosition.y_pos.to<double>());

}


units::second_t SwerveDrive::getDeltaTime() {

    m_timer.Stop();
    units::second_t timePassed = m_timer.Get();
    m_timer.Reset();
    m_timer.Start();
    return timePassed;
}


void SwerveDrive::updateOdometry(units::radians_per_second_t angularSpeed, units::radian_t currentGyroAngle) {

    // units::radian_t deltaAngle{currentGyroAngle - m_previousAngle};

    // m_previousAngle = currentGyroAngle;

    // units::radians_per_second_t angularSpeed = deltaAngle/20_ms;

    Vector2D FrontLeftTang{units::math::atan2(m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(angularSpeed.to<double>()*m_radius.to<double>())};

    Vector2D FrontRightTang{units::math::atan2(-m_trackWidth/2, m_wheelBase/2) + 90_deg, units::meter_t(angularSpeed.to<double>()*m_radius.to<double>())};

    Vector2D BottomLeftTang{units::math::atan2(m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(angularSpeed.to<double>()*m_radius.to<double>())};

    Vector2D BottomRightTang{units::math::atan2(-m_trackWidth/2, -m_wheelBase/2) + 90_deg, units::meter_t(angularSpeed.to<double>()*m_radius.to<double>())};


    //create a struct that holds all the vectors that are tangent to the drive base and have a magnitude of the rotational velocity given by the gyro
    // tangentVectors rotationalTangentVectors = getTangentVectors(units::meter_t(angularSpeed.to<double>()*m_radius.to<double>()));

    Vector2D* summedVectorFR = m_moduleFR->getDirection() - FrontRightTang;
    Vector2D* summedVectorFL = m_moduleFL->getDirection() - FrontLeftTang;
    Vector2D* summedVectorBR = m_moduleBR->getDirection() - BottomRightTang;
    Vector2D* summedVectorBL = m_moduleBL->getDirection() - BottomLeftTang;

    // Vector2D* summedVectorFR = m_moduleFR->getDirection() - rotationalTangentVectors.FrontRight;
    // Vector2D* summedVectorFL = m_moduleFL->getDirection() - rotationalTangentVectors.FrontLeft;
    // Vector2D* summedVectorBR = m_moduleBR->getDirection() - rotationalTangentVectors.BackRight;
    // Vector2D* summedVectorBL = m_moduleBL->getDirection() - rotationalTangentVectors.BackLeft;

    //find the average of the direction and velocity vector
    // units::radian_t averageDirection = (summedVectorFR->getDirection() + summedVectorFL->getDirection() + summedVectorBR->getDirection() + summedVectorBL->getDirection())/4;
    units::meter_t averageVelocity = (summedVectorFR->getMagnitude() + summedVectorFL->getMagnitude() + summedVectorBR->getMagnitude() + summedVectorBL->getMagnitude())/4;
    

    summedVectorBL->setMagnitude(1_m);
    summedVectorBR->setMagnitude(1_m);
    summedVectorFL->setMagnitude(1_m);
    summedVectorFR->setMagnitude(1_m);

    Vector2D* averageDirectionVec = *summedVectorFR + *summedVectorFL;
    averageDirectionVec->setMagnitude(1_m);
    averageDirectionVec = *averageDirectionVec + *summedVectorBR;
    averageDirectionVec->setMagnitude(1_m);
    averageDirectionVec = *averageDirectionVec + *summedVectorBL;
    

    units::radian_t averageDirection {averageDirectionVec->getDirection()};

    // units::radian_t averageDirection = summedVectorBR->getDirection();
    // units::meter_t averageVelocity = summedVectorBR->getMagnitude();

    // delete vector pointers
    delete summedVectorFR;
    delete summedVectorFL;
    delete summedVectorBL;
    delete summedVectorBR;

    Vector2D* travelVelocity = new Vector2D(averageDirection, averageVelocity);

    // std::cout << "Direction Tang: " << FrontRightTang.getDirection().to<double>()*(180/M_PI) << " Direction Velocity: " << m_moduleFR->getDirection().getDirection().to<double>()*(180/M_PI) << std::endl;
    // std::cout << "Magnitude Tang: " << FrontRightTang.getMagnitude().to<double>() << " Velocity Magnitude: " << m_moduleFR->getDirection().getMagnitude().to<double>() << std::endl;

    //offsets the travel direction vector by the current gyro angle to allow for field orientated x and y movement otherwise its robot relative
    travelVelocity->setDirection(travelVelocity->getDirection() + currentGyroAngle);

    // std::cout << "Direction of Travel: " << summedVector->getDirection().to<double>()*(180/M_PI) << " Speed of travel: " << summedVector->getMagnitude().to<double>() << std::endl;

    if (travelVelocity->getMagnitude() < 0.001_m) {
        travelVelocity->setMagnitude(0_m);
    }

    // std::cout << "Angular speed: " << angularSpeed.to<double>()*180/M_PI << std::endl;
    // std::cout << "Direction: " << travelVelocity->getDirection().to<double>()*180/M_PI << " Speed: " << travelVelocity->getMagnitude().to<double>() << std::endl;

    units::second_t deltaTime = getDeltaTime();
    // std::cout << units::math::atan2(0_m - m_currentPosition.y_pos, 0_m - m_currentPosition.x_pos).to<double>()*180/M_PI
    //  << std::endl;
    // std::cout << "angular Speed : " << angularSpeed.to<double>()*m_radius.to<double>()  << " travel Velocity: " << m_moduleFL->getDirection().getMagnitude().to<double>() << std::endl;

    //add the change in position to the current position
    // m_currentPosition.x_pos = units::meter_t(travelVelocity->getX().to<double>()*0.02) + m_currentPosition.x_pos;
    // m_currentPosition.y_pos = units::meter_t(travelVelocity->getY().to<double>()*0.02) + m_currentPosition.y_pos;

    m_currentPosition.x_pos = units::meter_t(travelVelocity->getX().to<double>()*deltaTime.to<double>()) + m_currentPosition.x_pos;
    m_currentPosition.y_pos = units::meter_t(travelVelocity->getY().to<double>()*deltaTime.to<double>()) + m_currentPosition.y_pos;


}


