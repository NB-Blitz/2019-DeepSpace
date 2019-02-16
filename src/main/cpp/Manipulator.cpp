#include "Manipulator.hpp"
#include "math.h"
#include <iostream>

//Pete's Roborio is Team Number 5150
frc::Manipulator::Manipulator() :
    Shoulder_Motor(3), 
    Shoulder_Motor_Limit_Switch(0),
    Elbow_Motor(2),
    Elbow_Motor_Limit_Switch(1)//Placeholder ID
    //Wrist_Axis(10), //Placeholder ID
    //Wrist_Axis_Limit_Switch(2)//Placeholder ID
    /*
    Shoulder_Pot(0, 360, SHOULDER_POT_OFFSET),
    Elbow_Pot(1, 360, ELBOW_POT_OFFSET),
    Wrist_Pot(2, 360, WRIST_POT_OFFSET)
    */
{

}
//Main has opposite direction to secondary (in degrees)
void frc::Manipulator::manipSet(double speed, int axisID, bool areLimits, double rawHome) //Moves Manipulator in accordance to the joystick without PID
{
    if (areLimits)
    {
        if (axisID == Shoulder_Axis)
        {
            bool cancel1 = ((getDegrees(Shoulder_Axis) >= MAX_RANGE_SHOULDER) && speed > 0);
            bool cancel2 = (isLimit(Shoulder_Axis) && speed < 0);
            if (!(cancel1 || cancel2))
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
            bool cancel1 = ((getDegrees(Elbow_Axis) >= MAX_RANGE_ELBOW) && speed > 0);
            bool cancel2 = (isLimit(Elbow_Axis) && speed < 0);
            if (!(cancel1 || cancel2))
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
        /*
        else if (axisID == Wrist_Axis)
        {
            bool cancel1 = ((getDegrees(Wrist_Axis) >= MAX_RANGE_WRIST) && speed > 0);
            bool cancel2 = (isLimit(Wrist_Axis) && speed < 0);
            if (!(cancel1 || cancel2))
            {
                Wrist_Axis.Set(ControlMode::PercentOutput, speed); 
            }
            else
            {
                Wrist_Axis.Set(ControlMode::PercentOutput, Off);
            }
            frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
            frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
        }
        */
    
    }
    else
    {
        if (axisID == Shoulder_Axis)
        {
            bool cancel1 = ((getDegrees(Shoulder_Axis, rawHome) >= MAX_RANGE_SHOULDER) && speed > 0);
            bool cancel2 = ((getDegrees(Shoulder_Axis, rawHome)) <= DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER && speed < 0);
            if (!(cancel1 || cancel2))
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
            bool cancel2 = ((getDegrees(Elbow_Axis, rawHome)) <= DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_ELBOW && speed < 0);
            if (!(cancel1 || cancel2))
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
        /*
        else if (axisID == Wrist_Axis)
        {
            bool cancel1 = ((getDegrees(Wrist_Axis, rawHome) >= MAX_RANGE_WRIST) && speed > 0);
            bool cancel2 = (getRawUnits(Wrist_Axis) < rawHome && speed < 0);
            if (!(cancel1 || cancel2))
            {
                Wrist_Axis.Set(ControlMode::PercentOutput, speed); 
            }
            else
            {
                Wrist_Axis.Set(ControlMode::PercentOutput, Off);
            }
            frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
            frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
        }
        */
    }
    
}

void frc::Manipulator::manipSetToDegrees(double degrees, int axisID, bool areLimits, double rawHome)
{
    double currentDegrees;
    if (areLimits)
    {
        currentDegrees = getDegrees(axisID);
        if (axisID == Shoulder_Axis) //Prevents movement to unsafe areas
        {
            double speed = .25;//(1 - (1 / (abs((degrees-currentDegrees) * 0.03) + 1)) * .3) + .1;
            if ((degrees - currentDegrees) > 1.5)
            {
                manipSet(speed, Shoulder_Axis);
            }
            else if ((degrees - currentDegrees) < -1.5)
            {
                manipSet(-speed, Shoulder_Axis);
            }
            else
            {
                manipSet(Off, Shoulder_Axis);
            }
        }
        else if (axisID == Elbow_Axis)
        {
            double speed = .25;//(1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1)) * .3) + .1;
            if ((degrees - currentDegrees) > 0.5)
            {
                manipSet(speed, Elbow_Axis);
            }
            else if ((degrees - currentDegrees) < -0.5)
            {
                manipSet(-speed, Elbow_Axis);
            }
            else
            {
                manipSet(Off, Elbow_Axis);
            }
        }
        else
        {
            manipSet(Off, axisID);
        }
    }
    else
    {
        currentDegrees = getDegrees(axisID, rawHome);
        if (axisID == Shoulder_Axis) //Prevents movement to unsafe areas
        {
            double speed = .25;//(1 - (1 / (abs((degrees-currentDegrees) * 0.03) + 1)) * .3) + .1;
            if ((degrees - currentDegrees) > 1.5)
            {
                manipSet(speed, Shoulder_Axis, false, rawHome);
            }
            else if ((degrees - currentDegrees) < -1.5)
            {
                manipSet(-speed, Shoulder_Axis, false, rawHome);
            }
            else
            {
                manipSet(Off, Shoulder_Axis, false, rawHome);
            }
        }
        else if (axisID == Elbow_Axis)
        {
            double speed = .25;//(1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1)) * .3) + .1;
            if ((degrees - currentDegrees) > 1.5)
            {
                manipSet(speed, Elbow_Axis, false, rawHome);
            }
            else if ((degrees - currentDegrees) < -1.5)
            {
                manipSet(-speed, Elbow_Axis, false, rawHome);
            }
            else
            {
                manipSet(Off, Elbow_Axis, false, rawHome);
            }
        }
    }
}

bool frc::Manipulator::isLimit(int axisID) //Returns if a limit switch is activated - Opposite logic due to wiring
{
    if (axisID == Shoulder_Axis)
    {
        return !Shoulder_Motor_Limit_Switch.Get();
    }
    else if (axisID == Elbow_Axis)
    {
        return !Elbow_Motor_Limit_Switch.Get();   
    }
    /*
    else if (axisID == Wrist_Axis)
    {
        return !Wrist_Axis_Limit_Switch.Get();   
    }
    */
    return true;
}

double frc::Manipulator::getRawUnits(int axisID)
{
    if (axisID == Shoulder_Axis)
    {
        return Shoulder_Motor.GetSelectedSensorPosition(0);
    }
    else if (axisID == Elbow_Axis)
    {
        return Elbow_Motor.GetSelectedSensorPosition(0);
    }
    /*
    else if (axisID == Wrist_Axis)
    {
        return Wrist_Motor.GetSelectedSensorPosition(0);
    }
    */
    else
    {
        return 0;
    }
    
}

double frc::Manipulator::getDegrees(int axisID, double rawHome) //Returns degrees from an encoder converting from raw counts
{
    if (axisID == Shoulder_Axis)
    {
        double degrees = (getRawUnits(Shoulder_Axis) - rawHome) / TO_DEGREES_SHOULDER;
        return abs(fmod(DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER + degrees, 360));   
    }
    else if (axisID == Elbow_Axis) //Needs changing
    {
        double degrees = (rawHome - getRawUnits(Elbow_Axis)) / TO_DEGREES_ELBOW;
        return abs(fmod(DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_ELBOW + degrees, 360));   
    }
    /*
    else if (axisID == Wrist_Axis)
    {
        double degrees = (rawHome + getRawUnits(Wrist_Axis) / TO_DEGREES_WRIST;
        return abs(fmod(degrees, 360)) + DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST;   
    }*/

    /*
    if (axisID == Shoulder_Axis)
    {
        return Shoulder_Pot.Get();
    }
    else if (axisID == Elbow_Axis)
    {
        return Elbow_Pot.Get();
    }
    else if (axisID == Wrist_Axis)
    {
        return Wrist_Pot.Get();
    }

    */
    else
    {
        return 0;
    }   
}

void frc::Manipulator::resetDegrees(int axisID) //Resets the encoder to 0 degrees
{
    if (axisID == Shoulder_Axis)
    {
        Shoulder_Motor.SetSelectedSensorPosition(0, 0, 0);
    }
    else if (axisID == Elbow_Axis)
    {
        Elbow_Motor.SetSelectedSensorPosition(0, 0, 0);
    }
    /*
    else if (axisID == Wrist_Axis)
    {
        //Wrist_Axis.SetSelectedSensorPosition(0, 0, 0);
    }
    */
   
}

bool frc::Manipulator::resetToEncoder(int axisID)
{
    if (!isLimit(axisID))
    {
        manipSet(-0.35 , axisID);
        return false;
    }
    else
    {
        resetDegrees(axisID);
        manipSet(Off, axisID);
        return true;
    }
}
//Positive = down, negative = up

void frc::Manipulator::initializePID(bool firstTime) //Initializes the PID functions
{
    if (firstTime)
    {
        Shoulder_Motor.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Shoulder_Motor.SetSensorPhase(true);
        Shoulder_Motor.ConfigNominalOutputForward(0, 30);
        Shoulder_Motor.ConfigNominalOutputReverse(0, 30);
        Shoulder_Motor.ConfigPeakOutputForward(1, 30);
        Shoulder_Motor.ConfigPeakOutputReverse(-1, 30);
    }   
    Shoulder_Motor.Config_kP(0, PID_P_SHOULDER, 30);
    Shoulder_Motor.Config_kI(0, PID_I_SHOULDER, 30);
    Shoulder_Motor.Config_kD(0, PID_D_SHOULDER, 30);
    Shoulder_Motor.Config_kF(0, PID_F_SHOULDER, 30);
    if (firstTime)
    {
        Elbow_Motor.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Elbow_Motor.SetSensorPhase(true);
        Elbow_Motor.ConfigNominalOutputForward(0, 30);
        Elbow_Motor.ConfigNominalOutputReverse(0, 30);
        Elbow_Motor.ConfigPeakOutputForward(1, 30);
        Elbow_Motor.ConfigPeakOutputReverse(-1, 30);
    }   
    Elbow_Motor.Config_kP(0, PID_P_ELBOW, 30);
    Elbow_Motor.Config_kI(0, PID_I_ELBOW, 30);
    Elbow_Motor.Config_kD(0, PID_D_ELBOW, 30);
    Elbow_Motor.Config_kF(0, PID_F_ELBOW, 30);
   /*
    if (firstTime)
    {
        Wrist_Axis.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Wrist_Axis.SetSensorPhase(true);
        Wrist_Axis.ConfigNominalOutputForward(0, 30);
        Wrist_Axis.ConfigNominalOutputReverse(0, 30);
        Wrist_Axis.ConfigPeakOutputForward(1, 30);
        Wrist_Axis.ConfigPeakOutputReverse(-1, 30);
    }   
    Wrist_Axis.Config_kP(0, PID_P_WRIST, 30);
    Wrist_Axis.Config_kI(0, PID_I_WRIST, 30);
    Wrist_Axis.Config_kD(0, PID_D_WRIST, 30);
    Wrist_Axis.Config_kF(0, PID_F_WRIST, 30);
    */
}

double frc::Manipulator::getP(int axisID) //Returns the Proportional Multiplier of PID
{
    if (axisID == Shoulder_Axis)
    {
        return PID_P_SHOULDER;
    }
    else if (axisID == Elbow_Axis)
    {
        return PID_P_ELBOW;
    }
   /*
    else if (axisID == Wrist_Axis)
    {
        return PID_P_WRIST;
    }
    */
    else
    {
        return -1;
    }
}

double frc::Manipulator::getI(int axisID) //Returns the Integral Multiplier of PID
{
    if (axisID == Shoulder_Axis)
    {
        return PID_I_SHOULDER;
    }
    else if (axisID == Elbow_Axis)
    {
        return PID_I_ELBOW;
    }
    /*
    else if (axisID == Wrist_Axis)
    {
        return PID_I_WRIST;
    }
    */
    else
    {
        return -1;
    }
}

double frc::Manipulator::getD(int axisID) //Returns the Differential Multiplier of PID
{
    if (axisID == Shoulder_Axis)
    {
        return PID_D_SHOULDER;
    }
    else if (axisID == Elbow_Axis)
    {
        return PID_D_ELBOW;
    }
   /*
    else if (axisID == Wrist_Axis)
    {
        return PID_D_WRIST;
    }
    */
    else
    {
        return -1;
    }
}

double frc::Manipulator::getF(int axisID) //Returns the FeedForward Multiplier of PID
{
    if (axisID == Shoulder_Axis)
    {
        return PID_F_SHOULDER;
    }
    else if (axisID == Elbow_Axis)
    {
        return PID_F_ELBOW;
    }
   /*
    else if (axisID == Wrist_Axis)
    {
        return PID_F_WRIST;
    }
    */
    else
    {
        return -1;
    }
}

//Angles are relative to true zero degrees (second axis 0 degrees means it moves in opposite direction of main axis)
double frc::Manipulator::getAngleForCoordinates(double x, double y, int axisID) //Returns the angle of an axis necessary for reaching the given coordinates
{
    double pi = 3.14159265358979323846;
    double radiansToDegrees = (180 / pi);
    double c = sqrt(pow(x, 2) + pow(y, 2));
    if (axisID == Shoulder_Axis)
    {
        double d1 = acos((pow(LENGTH_SHOULDER,2) + pow(c,2) - pow(LENGTH_ELBOW,2)) / (2 * LENGTH_SHOULDER * c));
        double d2 = atan2(y,x);
        return ((d1 + d2) * radiansToDegrees) + DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER;
        
    }
    else if (axisID == Elbow_Axis)
    {
        return (acos((pow(LENGTH_SHOULDER,2) + pow(LENGTH_ELBOW,2) - pow(c,2)) / (2 * LENGTH_SHOULDER * LENGTH_ELBOW)) * radiansToDegrees); //180 - might be wrong
    }
    else
    {   
        return 0;
    }
    /*
        all functions are in radians -> I return degrees

    */
}

void frc::Manipulator::moveToCoordinates(double x, double y, bool areLimits, double rawHomeShoulder, double rawHomeElbow) //Moves both axes to the angles necessary to reach the given coordinates
{
    if (isPossible(x,y))
    {
        if (areLimits)
        {
            manipSetToDegrees(getAngleForCoordinates(x,y,Shoulder_Axis), Shoulder_Axis);
            manipSetToDegrees(getAngleForCoordinates(x,y,Elbow_Axis), Elbow_Axis);
        }
        else
        {

            manipSetToDegrees(getAngleForCoordinates(x,y,Shoulder_Axis), Shoulder_Axis, false, rawHomeShoulder);
            manipSetToDegrees(getAngleForCoordinates(x,y,Elbow_Axis), Elbow_Axis, false, rawHomeElbow);
        }
    }  
}
bool frc::Manipulator::isPossible(double x, double y)
{
    double angle1 = getAngleForCoordinates(x,y,Shoulder_Axis);
    double angle2 = getAngleForCoordinates(x,y,Elbow_Axis);
    //Checks if angles are attainable
    if (((angle1 < MAX_RANGE_SHOULDER) && (angle1 > DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER)) && ((angle1 < MAX_RANGE_SHOULDER) && (angle1 > DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER)))
    {
        //Checks if distance is too far (will incur penalties)
        return true;
        /*
        if (x < -30)
        {
            return true;
        }
        */
        
    }
    return false;
}
/*
double frc::Manipulator::getAngleForParallel(double x, double y)
{
    double angle1 = getAngleForCoordinates(x,y,0);
    double angle2 = getAngleForCoordinates(x,y,1);
    return (360 - angle1 - angle2 - DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST);    
}

void frc::Manipulator::moveToParallel(double x, double y)
{
    manipSetPID(getAngleForParallel(x,y), 2);
}
*/