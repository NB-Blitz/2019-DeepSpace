#include "Manipulator.hpp"

Blitz::Manipulator::Manipulator() :
ArmsIn(2),
ArmsOut(3)
{
    
}

void Blitz::Manipulator::CloseArms()
{
    ArmsIn.Set(true);
    ArmsOut.Set(false);
}

void Blitz::Manipulator::OpenArms()
{
    ArmsIn.Set(false);
    ArmsOut.Set(true);
}