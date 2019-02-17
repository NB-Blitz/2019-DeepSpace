#include "Climber.hpp"

Blitz::Climber::Climber():
    FrontUpSolenoid(0),
    FrontDownSolenoid(1),
    BackUpSolenoid(2),
    BackDownSolenoid(3),
    comp(0)
{

}

void Blitz::Climber::StartCompressor()
{
    comp.SetClosedLoopControl(true);
}

void Blitz::Climber::SetFrontSolenoid(bool Up)
{
    FrontUpSolenoid.Set(Up);
    FrontDownSolenoid.Set(!Up);
}

void Blitz::Climber::SetBackSolenoid(bool Up)
{
    BackUpSolenoid.Set(Up);
    BackDownSolenoid.Set(!Up);
}