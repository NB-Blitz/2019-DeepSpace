#include "FieldOrientedControl.hpp"

Blitz::Models::MecanumInput Blitz::FieldOrientedControl::FieldControl(double x, double y, double Yaw)
{
    float pi = 3.1415926; 
    double angle = 0;
    double angleR = ((Yaw + 180) * pi) / 180.0;
    double angleJ = atan2(y, x) + pi;
    double m = sqrt(pow(x,2) + pow(y,2)); //Delcaring r (magnitude); 
    frc::SmartDashboard::PutNumber("Angle",angleR);

    //For Quadrant 4
    if(angleJ > 90 && angleJ < 270)
    {
      angle = pi + angleJ;
    }
    else if(angleJ > 270)
    {
      angle = (2 *pi) + angleJ;
    }

    else
    {
      angle = angleJ;
    }
    
    angle += angleR;

    Blitz::Models::MecanumInput Input(m * cos(angle), m * sin(angle), 0);
    return Input;
}
