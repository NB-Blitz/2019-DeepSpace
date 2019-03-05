#include "Climber.hpp"

Blitz::Climber::Climber():
    BackSolenoid(0),
    comp(0)
{

}

void Blitz::Climber::StartCompressor()
{
    comp.SetClosedLoopControl(true);
}

void Blitz::Climber::SetBackSolenoid(bool Up)
{
    BackSolenoid.Set(Up);
}