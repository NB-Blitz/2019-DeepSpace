#include "Blitz_Joystick.hpp"

frc::Blitz_Joystick::Blitz_Joystick() :
    Joystick_Main(0),
    Joystick_Secondary(1)
{

}

bool frc::Blitz_Joystick::getButton(int id, int axisID)
{
    if (axisID == 0)
    {
        return Joystick_Main.GetRawButton(id);
    }
    else if (axisID == 1)
    {
        return Joystick_Secondary.GetRawButton(id);
    }
    return false;
}

double frc::Blitz_Joystick::getAxis(int id, int axisID)
{
    if (axisID == 0)
    {
        return Joystick_Main.GetRawAxis(id);
    }
    else if (axisID == 1)
    {
        return Joystick_Secondary.GetRawAxis(id);
    }
    return 0;
}
