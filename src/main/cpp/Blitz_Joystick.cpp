#include "Blitz_Joystick.hpp"
#include "frc/WPILib.h"

frc::Blitz_Joystick::Blitz_Joystick() :
    joystick(0)
{

}

int frc::Blitz_Joystick::getJoystick()
{
    double joyVal = joystick.GetRawAxis(1);
    if (joyVal > JOYSTICK_DEAD_ZONE)
    {
        return 1;
    }
    else if (joyVal < -JOYSTICK_DEAD_ZONE)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

bool frc::Blitz_Joystick::getButton(int id)
{
    return joystick.GetRawButton(id);
}