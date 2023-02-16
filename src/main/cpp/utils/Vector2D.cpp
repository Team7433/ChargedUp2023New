#include "utils/Vector2D.h"


using namespace iona;

Vector2D::Vector2D(units::radian_t direction, units::meter_t magnitude) {
    setPolar(direction, magnitude);
}

Vector2D::Vector2D(units::meter_t x_magnitude, units::meter_t y_magnitude) {
    setCartesian(x_magnitude, y_magnitude);
}

void Vector2D::updateCartesianFromPolar() {

    m_xPos = m_magnitude * units::math::cos(m_direction);
    m_yPos = m_magnitude * units::math::sin(m_direction);

}

void Vector2D::updatePolarFromCartesian() {

    if(m_xPos == 0_m && m_yPos == 0_m) {
        m_direction = 0_rad;
        m_magnitude = 0_m;    
        return;
    }

    m_direction = units::math::atan2(m_yPos, m_xPos);
    m_magnitude = units::meter_t( sqrt( pow(m_xPos.to<double>(), 2) + pow(m_yPos.to<double>(), 2) ) );

}

void Vector2D::setX(units::meter_t x_magnitude) {
    setCartesian(x_magnitude, m_yPos);
}


void Vector2D::setY(units::meter_t y_magnitude) {
    setCartesian(m_xPos, y_magnitude);
}

void Vector2D::setDirection(units::radian_t direction) {
    setPolar(direction, m_magnitude);
}

void Vector2D::setMagnitude(units::meter_t magnitude) {
    setPolar(m_direction, magnitude);
}

void Vector2D::setCartesian(units::meter_t x_magnitude, units::meter_t y_magnitude) {
    m_xPos = x_magnitude;
    m_yPos = y_magnitude;

    updatePolarFromCartesian();
}

void Vector2D::setPolar(units::radian_t direction, units::meter_t magnitude) {

    //if the magnitude is negative then flip the direction of the vector
    m_direction = std::signbit(magnitude.to<double>()) ? direction + 180_deg : direction;
    m_magnitude = std::signbit(magnitude.to<double>()) ? -magnitude : magnitude;

    updateCartesianFromPolar();
}

Vector2D* Vector2D::operator+ (const Vector2D& vector) {
    Vector2D* newVector =  new Vector2D(this->getX() + vector.getX(), this->getY() + vector.getY());
    return newVector;
    
}

Vector2D* Vector2D::operator- (const Vector2D& vector) {
    Vector2D* newVector =  new Vector2D(this->getX() - vector.getX(), this->getY() - vector.getY());
    return newVector;
    
}