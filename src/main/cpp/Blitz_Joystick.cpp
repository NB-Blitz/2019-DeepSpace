#include "Blitz_Joystick.hpp"

frc::Blitz_Joystick::Blitz_Joystick() :
    joystick(0)
{

}

bool frc::Blitz_Joystick::getButton(int id)
{
    return joystick.GetRawButton(id);
}

double frc::Blitz_Joystick::getAxis(int id)
{
    return joystick.GetRawAxis(id);
}
