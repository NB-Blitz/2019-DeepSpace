#include "Manipulator.hpp"
#include "math.h"

Blitz::Manipulator::Manipulator() :
    Shoulder_Motor(4),
    Elbow_Motor(5),    
    Wrist_Motor(6)

{

}
//Main has opposite direction to secondary (in degrees)
void Blitz::Manipulator::manipSet(double speed, int axisID, double rawHome) //Moves Manipulator in accordance to the joystick without PID
{
    if (axisID == Shoulder_Axis)
    {
        bool cancel1 = ((getDegrees(Shoulder_Axis, rawHome) >= MAX_RANGE_SHOULDER) && speed > 0);
        bool cancel2 = ((getDegrees(Shoulder_Axis, rawHome) <= MIN_RANGE_SHOULDER) && speed < 0);
        if(!(cancel1 || cancel2) && (abs(speed)) > 0.1)
        {
            Shoulder_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    else if (axisID == Elbow_Axis)
    {
        bool cancel1 = ((getDegrees(Elbow_Axis, rawHome) >= MAX_RANGE_ELBOW) && speed > 0);
        bool cancel2 = ((getDegrees(Elbow_Axis, rawHome) <= MIN_RANGE_ELBOW) && speed < 0);
        if (!(cancel1 || cancel2) && (abs(speed)) > 0.1)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    else if (axisID == Wrist_Axis)
    {
        bool cancel1 = ((getDegrees(Wrist_Axis, rawHome) >= MAX_RANGE_WRIST) && speed > 0);
        bool cancel2 = ((getDegrees(Wrist_Axis, rawHome) <= MIN_RANGE_WRIST) && speed < 0);
        if (!(cancel1 || cancel2) && (abs(speed)) > 0.1)
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
}

void Blitz::Manipulator::manipSetToDegrees(double degrees, int axisID, double rawHome)
{
    double currentDegrees;
    currentDegrees = getDegrees(axisID, rawHome);
    if (axisID == Shoulder_Axis) //Prevents movement to unsafe areas
    {
        double speed = (1 - (1 / (abs((degrees-currentDegrees) * 0.03) + 1)) * .3) + .1;
        if ((degrees - currentDegrees) > 1.5)
        {
            manipSet(speed, Shoulder_Axis, rawHome);
        }
        else if ((degrees - currentDegrees) < -1.5)
        {
            manipSet(-speed, Shoulder_Axis, rawHome);
        }
        else
        {
            manipSet(Off, Shoulder_Axis, rawHome);
        }
    }
    else if (axisID == Elbow_Axis)
    {
        double speed = (1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1)) * .3) + .1;
        if ((degrees - currentDegrees) > 1.5)
        {
            manipSet(speed, Elbow_Axis, rawHome);
        }
        else if ((degrees - currentDegrees) < -1.5)
        {
            manipSet(-speed, Elbow_Axis, rawHome);
        }
        else
        {
            manipSet(Off, Elbow_Axis, rawHome);
        }
    }
}

double Blitz::Manipulator::getRawUnits(int axisID)
{
    if (axisID == Shoulder_Axis)
    {
        return Shoulder_Motor.GetSelectedSensorPosition(0);
    }
    else if (axisID == Elbow_Axis)
    {
        return Elbow_Motor.GetSelectedSensorPosition(0);
    }
    else if (axisID == Wrist_Axis)
    {
        return Wrist_Motor.GetSelectedSensorPosition(0);
    }
    else
    {
        return 0;
    }
}

double Blitz::Manipulator::getDegrees(int axisID, double rawHome) //Returns degrees from an encoder converting from raw counts
{
    if (axisID == Shoulder_Axis)
    {
        double degrees = (getRawUnits(Shoulder_Axis) - rawHome) / TO_DEGREES_SHOULDER;
        return abs(HOME_POSITION_SHOULDER + degrees); //-180 is offset   
    }
    else if (axisID == Elbow_Axis) //Needs changing
    {
        double degrees = (rawHome - getRawUnits(Elbow_Axis)) / TO_DEGREES_ELBOW;
        return abs(fmod(HOME_POSITION_ELBOW + degrees, 360));   
    }
    else if (axisID == Wrist_Axis) //Up = Increase in count
    {
        double degrees = (getRawUnits(Wrist_Axis) - rawHome) / TO_DEGREES_WRIST;
        return abs(fmod(HOME_POSITION_WRIST - degrees, 360));   
    }
    else
    {
        return 0;
    }   
}

void Blitz::Manipulator::resetDegrees(int axisID) //Resets the encoder to 0 degrees
{
    if (axisID == Shoulder_Axis)
    {
        Shoulder_Motor.SetSelectedSensorPosition(0, 0, 0);
    }
    else if (axisID == Elbow_Axis)
    {
        Elbow_Motor.SetSelectedSensorPosition(0, 0, 0);
    }
    
    else if (axisID == Wrist_Axis)
    {
        Wrist_Motor.SetSelectedSensorPosition(0, 0, 0);
    }  
}

//Angles are relative to true zero degrees (second axis 0 degrees means it moves in opposite direction of main axis)
double Blitz::Manipulator::getAngleForCoordinates(double x, double y, int axisID) //Returns the angle of an axis necessary for reaching the given coordinates
{
    double pi = 3.14159265358979323846;
    double radiansToDegrees = (180 / pi);
    double c = sqrt(pow(x, 2) + pow(y, 2));
    if (axisID == Shoulder_Axis)
    {
        double d1 = acos((pow(LENGTH_SHOULDER,2) + pow(c,2) - pow(LENGTH_ELBOW,2)) / (2 * LENGTH_SHOULDER * c));
        double d2 = atan2(y,x);
        return ((d1 + d2) * radiansToDegrees) + MIN_RANGE_SHOULDER;
        
    }
    else if (axisID == Elbow_Axis)
    {
        return (acos((pow(LENGTH_SHOULDER,2) + pow(LENGTH_ELBOW,2) - pow(c,2)) / (2 * LENGTH_SHOULDER * LENGTH_ELBOW)) * radiansToDegrees); //180 - might be wrong
    }
    else
    {   
        return 0;
    }
        //all functions are in radians -> I return degrees
}

void Blitz::Manipulator::moveToCoordinates(double x, double y, double rawHomeShoulder, double rawHomeElbow) //Moves both axes to the angles necessary to reach the given coordinates
{
    if (isPossible(x,y))
    {
        manipSetToDegrees(getAngleForCoordinates(x,y,Shoulder_Axis), Shoulder_Axis, rawHomeShoulder);
        manipSetToDegrees(getAngleForCoordinates(x,y,Elbow_Axis), Elbow_Axis, rawHomeElbow);
    }  
}

bool Blitz::Manipulator::isPossible(double x, double y)
{
    double angle1 = getAngleForCoordinates(x,y,Shoulder_Axis);
    double angle2 = getAngleForCoordinates(x,y,Elbow_Axis);
    //Checks if angles are attainable
    if (((angle1 < MAX_RANGE_SHOULDER) && (angle1 > MIN_RANGE_SHOULDER)) && ((angle1 < MAX_RANGE_SHOULDER) && (angle1 > MIN_RANGE_SHOULDER)))
    {
        //Checks if distance is too far (will incur penalties)
        if (x < 40 && x > -54) //10 + 30 clearance or 24 + 30 clearance
        {
            return true;
        }
    }
    return false;
}


//May need to be checked
void Blitz::Manipulator::moveToXDegreesBelowParallel(double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist, double x)
{
    double degrees1 = getDegrees(Shoulder_Axis, rawHomeShoulder);
    double degrees2 = getDegrees(Elbow_Axis, rawHomeElbow);
    manipSetToDegrees(360 - degrees1 - degrees2 + x - MIN_RANGE_WRIST, Wrist_Axis, rawHomeWrist);
}
