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

    if(speed < 0)
    {
        currentPosition += PositionCounter.Get() * direction;
        PositionCounter.Reset();

        direction = -1;
    }
    else if(speed > .05)
    {
        currentPosition += PositionCounter.Get() * direction;
        PositionCounter.Reset();

        direction = 1;
    }
}

void Blitz::Manipulator::MoveManipulatorPosition(double diameter)
{
    //diameter = 19.575 - diameter;


    double angle = ((asin((((diameter/2) - 5.375)/4.25))*(180/3.1459))-90);

    
    cout << angle << endl;

    int counts = angle * PULSES_PER_ANGLE_SMALL_GEAR;

    //int counts = 30;

    currentPosition += PositionCounter.Get() * direction;
    PositionCounter.Reset();

    direction = -1;

    if(currentPosition > counts)
    {
        ClawTalon.Set(ControlMode::PercentOutput, -.5);
        direction = -1;
    }
    else if(currentPosition < counts)
    {
        ClawTalon.Set(ControlMode::PercentOutput, .5);
        direction = 1;
    }
    else 
    {
        ClawTalon.Set(ControlMode::PercentOutput, 0);
    }
    
}