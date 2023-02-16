#pragma once


#include <units/length.h>
#include <units/angle.h>

#include <units/math.h>
#include <math.h>


namespace iona {


class Vector2D {

 public:
    Vector2D(units::radian_t direction, units::meter_t magnitude);
    Vector2D(units::meter_t x_magnitude, units::meter_t y_magnitude);
 
    units::meter_t getX() const {return m_xPos;}
    units::meter_t getY() const {return m_yPos;}

    void setX(units::meter_t x_magnitude);
    void setY(units::meter_t y_magnitude);
    void setCartesian(units::meter_t x_magnitude, units::meter_t y_magnitude);

    units::meter_t getMagnitude() const {return m_magnitude;}
    units::radian_t getDirection() const {return m_direction;}

    void setMagnitude(units::meter_t magnitude);
    void setDirection(units::radian_t direction);
    void setPolar(units::radian_t direction, units::meter_t magnitude);

    Vector2D* operator+(const Vector2D& vector);
    Vector2D* operator-(const Vector2D& vector);

 private:
    //updates the cartesian coordinates from the current polar coordinates
    void updateCartesianFromPolar();

    //updates the polar coordinates from the current cartesian coordinates
    void updatePolarFromCartesian();
    
    units::meter_t m_xPos;
    units::meter_t m_yPos;

    units::radian_t m_direction;
    units::meter_t m_magnitude;












};


}