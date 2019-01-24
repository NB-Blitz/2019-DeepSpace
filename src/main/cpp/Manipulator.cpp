#include "Manipulator.hpp"
#include "frc/WPILib.h"

frc::Manipulator::Manipulator():
/*
    Chuck(0),
    Wrist(1),
    Chuck_Encoder(0,1),
    Wrist_Encoder(2,3),
*/	
    Main_Axis(0),
    Main_Axis_Limit_Switch(0)
{

}

double frc::Manipulator::getDegrees() //From Main_Axis
{
    return Main_Axis.GetPosition() * TO_DEGREES;
}

void frc::Manipulator::reset()
{
    if (!Main_Axis_Limit_Switch.Get())
    {
        Main_Axis.Set(-0.2);
    }
}

void frc::Manipulator::moveTo(double degrees) //0 degrees at limit switch, clockwise = positive change in degrees
{
    double currentDegrees = getDegrees();
    if (currentDegrees > (degrees + DEAD_ZONE))
    {
        //Negative Motion
        Main_Axis.Set(-0.2);
    }
    else if (currentDegrees < (degrees - DEAD_ZONE))
    {
        //Positive Motion
        Main_Axis.Set(0.2);
    }
}
