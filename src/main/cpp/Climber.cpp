#include "Climber.hpp"

Blitz::Climber::Climber():
    FrontSolenoid(0),
    BackSolenoid(1),
    comp(0)
{

}

void Blitz::Climber::StartCompressor()
{
    comp.SetClosedLoopControl(true);
}

void Blitz::Climber::SetFrontSolenoid(bool Up)
{
    FrontSolenoid.Set(Up);
}

void Blitz::Climber::SetBackSolenoid(bool Up)
{
    BackSolenoid.Set(Up);
}