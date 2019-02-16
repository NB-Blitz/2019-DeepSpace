#include "Blitz_Joystick.hpp"

frc::Blitz_Joystick::Blitz_Joystick() :
    Joystick_Main(0),
    Joystick_Secondary(1),
    XBox(2)
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

bool frc::Blitz_Joystick::getXBoxButton(int id)
{
    return XBox.GetRawButton(id);
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

double frc::Blitz_Joystick::getXBoxAxis(int id)
{
    return XBox.GetRawAxis(id);
}