#include "Manipulator.hpp"

Blitz::Manipulator::Manipulator() :
LimitSwitch(0),
ClawTalon(4),
PositionCounter(1)
{

}

void Blitz::Manipulator::ResetPosition()
{
    if(LimitSwitch.Get())
    {
        ClawTalon.Set(ControlMode::PercentOutput, 1);
    }
    else
    {
        ClawTalon.Set(ControlMode::PercentOutput, 0);
        PositionCounter.Reset();
        currentPosition = 0;
    }
}

void Blitz::Manipulator::MoveManipulatorSpeed(double speed)
{
    ClawTalon.Set(ControlMode::PercentOutput, speed);
    
    frc::SmartDashboard::PutNumber("Counter", PositionCounter.Get());
}

void Blitz::Manipulator::MoveManipulatorPosition(double diameter)
{
    double angle = asin(((diameter/2) - 5.375)/4.25) - 90;

    int counts = angle * PULSES_PER_ANGLE_SMALL_GEAR;

    counts -= currentPosition;

    frc::SmartDashboard::PutNumber("Counter", PositionCounter.Get());

    if(PositionCounter.Get() < counts + 4)
    {
        ClawTalon.Set(ControlMode::PercentOutput, -1);
        direction = -1;
    }
    else if(PositionCounter.Get() > counts - 4)
    {
        ClawTalon.Set(ControlMode::PercentOutput, 1);
        direction = 1;
    }
    else 
    {
        currentPosition += PositionCounter.Get() * direction;
        ClawTalon.Set(ControlMode::PercentOutput, 0);
        PositionCounter.Reset();

    }
    
}