#include "Manipulator.hpp"
#include "math.h"
#include <iostream>

//Pete's Roborio is Team Number 5150
frc::Manipulator::Manipulator() :
    Main_Axis(3), 
    Main_Axis_Limit_Switch(0),
    Secondary_Axis(2),
    Secondary_Axis_Limit_Switch(1)//Placeholder ID
    //Wrist_Axis(10), //Placeholder ID
    //Wrist_Axis_Limit_Switch(2)//Placeholder ID
{

}
//Main has opposite direction to secondary (in degrees)
void frc::Manipulator::manipSet(double speed, int axisID) //Moves Manipulator in accordance to the joystick without PID
{
    if (axisID == 0)
    {
        bool cancel1 = ((getDegrees(0) >= MAX_RANGE_MAIN) && speed > 0);
        bool cancel2 = (isLimit(0) && speed < 0);
        if (!(cancel1 || cancel2))
        {
            Main_Axis.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Main_Axis.Set(ControlMode::PercentOutput, 0);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    else if (axisID == 1)
    {
        bool cancel1 = ((getDegrees(1) >= MAX_RANGE_SECONDARY) && speed > 0);
        bool cancel2 = (isLimit(1) && speed < 0);
        if (!(cancel1 || cancel2))
        {
            Secondary_Axis.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Secondary_Axis.Set(ControlMode::PercentOutput, 0);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
   /*
    else if (axisID == 2)
    {
        bool cancel1 = ((getDegrees(2) >= MAX_RANGE_WRIST) && speed > 0);
        bool cancel2 = (isLimit(2) && speed < 0);
        if (!(cancel1 || cancel2))
        {
            Wrist_Axis.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Wrist_Axis.Set(ControlMode::PercentOutput, 0);
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    */

   
}
//It works, at least for 45 degrees tested (further tuning may be needed)
void frc::Manipulator::manipSetPID(double degrees, int axisID) //Moves Manipulator to a set degree with PID
{
    //3649 per half rotation
    if (axisID == 0)
    {
        bool cancel1 = (degrees >= MAX_RANGE_MAIN);
        bool cancel2 = (isLimit(0) && (degrees < getDegrees(0)));
        double encoderCounts = (ENCODER_COUNTS_PER_ROTATION_MAIN / 180) * degrees;
        frc::SmartDashboard::PutNumber("PostEnc: ", encoderCounts);
        if (!(cancel1 || cancel2))
        {   
            frc::SmartDashboard::PutNumber("Should be same: ", encoderCounts);        
            Main_Axis.Set(ControlMode::Position, encoderCounts); 
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    else if (axisID == 1)
    {
        bool cancel1 = (degrees >= MAX_RANGE_SECONDARY);
        bool cancel2 = (isLimit(1) && (degrees < getDegrees(1)));
        double encoderCounts = (ENCODER_COUNTS_PER_ROTATION_SECONDARY / 180) * degrees;
        if (!(cancel1 || cancel2))
        {
            Secondary_Axis.Set(ControlMode::Position, encoderCounts); 
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }

   /*
    else if (axisID == 2)
    {
        bool cancel1 = (degrees >= MAX_RANGE_WRIST);
        bool cancel2 = (isLimit(2) && (degrees < getDegrees(2)));
        double encoderCounts = (ENCODER_COUNTS_PER_ROTATION_WRIST / 180) * degrees;
        if (!(cancel1 || cancel2))
        {
            Wrist_Axis.Set(ControlMode::Position, encoderCounts); 
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    */
}

bool frc::Manipulator::isLimit(int axisID) //Returns if a limit switch is activated - Opposite logic due to wiring
{
    if (axisID == 0)
    {
        return !Main_Axis_Limit_Switch.Get();
    }
    else if (axisID == 1)
    {
        return !Secondary_Axis_Limit_Switch.Get();   
    }
    /*
    else if (axisID == 2)
    {
        return !Wrist_Axis_Limit_Switch.Get();   
    }
    */
    return true;
}

double frc::Manipulator::getDegrees(int axisID) //Returns degrees from an encoder converting from raw counts
{
    if (axisID == 0)
    {
        double degrees =  Main_Axis.GetSelectedSensorPosition(0) / TO_DEGREES_MAIN;
        return abs(fmod(degrees + DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_MAIN, 360));   
    }
    else if (axisID == 1)
    {
        double degrees =  Secondary_Axis.GetSelectedSensorPosition(0) / TO_DEGREES_SECONDARY;
        return abs(fmod(degrees + DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SECONDARY, 360));   
    }
    /*
    else if (axisID == 2)
    {
        double degrees = Wrist_Axis.GetSelectedSensorPosition(0) / TO_DEGREES_WRIST;
        return abs(fmod(degrees, 360)) + DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST;   
    }*/
    else
    {
        return 0;
    }
    
}

void frc::Manipulator::resetDegrees(int axisID) //Resets the encoder to 0 degrees
{
    if (axisID == 0)
    {
        Main_Axis.SetSelectedSensorPosition(0, 0, 0);
    }
    else if (axisID == 1)
    {
        Secondary_Axis.SetSelectedSensorPosition(0, 0, 0);
    }
    /*
    else if (axisID == 1)
    {
        //Wrist_Axis.SetSelectedSensorPosition(0, 0, 0);
    }
    */
   
}

void frc::Manipulator::resetToEncoder(int axisID)
{
    if (!isLimit(axisID))
    {
        manipSet(-0.2 , axisID);
    }
    else
    {
        resetDegrees(axisID);
        manipSet(0, axisID);
    }
}
//Positive = down, negative = up

void frc::Manipulator::initializePID(bool firstTime) //Initializes the PID functions
{
    if (firstTime)
    {
        Main_Axis.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Main_Axis.SetSensorPhase(true);
        Main_Axis.ConfigNominalOutputForward(0, 30);
        Main_Axis.ConfigNominalOutputReverse(0, 30);
        Main_Axis.ConfigPeakOutputForward(1, 30);
        Main_Axis.ConfigPeakOutputReverse(-1, 30);
    }   
    Main_Axis.Config_kP(0, PID_P_MAIN, 30);
    Main_Axis.Config_kI(0, PID_I_MAIN, 30);
    Main_Axis.Config_kD(0, PID_D_MAIN, 30);
    Main_Axis.Config_kF(0, PID_F_MAIN, 30);
    if (firstTime)
    {
        Secondary_Axis.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Secondary_Axis.SetSensorPhase(true);
        Secondary_Axis.ConfigNominalOutputForward(0, 30);
        Secondary_Axis.ConfigNominalOutputReverse(0, 30);
        Secondary_Axis.ConfigPeakOutputForward(1, 30);
        Secondary_Axis.ConfigPeakOutputReverse(-1, 30);
    }   
    Secondary_Axis.Config_kP(0, PID_P_SECONDARY, 30);
    Secondary_Axis.Config_kI(0, PID_I_SECONDARY, 30);
    Secondary_Axis.Config_kD(0, PID_D_SECONDARY, 30);
    Secondary_Axis.Config_kF(0, PID_F_SECONDARY, 30);
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
    if (axisID == 0)
    {
        return PID_P_MAIN;
    }
    else if (axisID == 1)
    {
        return PID_P_SECONDARY;
    }
   /*
    else if (axisID == 2)
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
    if (axisID == 0)
    {
        return PID_I_MAIN;
    }
    else if (axisID == 1)
    {
        return PID_I_SECONDARY;
    }
    /*
    else if (axisID == 2)
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
    if (axisID == 0)
    {
        return PID_D_MAIN;
    }
    else if (axisID == 1)
    {
        return PID_D_SECONDARY;
    }
   /*
    else if (axisID == 2)
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
    if (axisID == 0)
    {
        return PID_F_MAIN;
    }
    else if (axisID == 1)
    {
        return PID_F_SECONDARY;
    }
   /*
    else if (axisID == 2)
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
    double c = sqrt(pow(LENGTH_MAIN, 2) + pow(LENGTH_SECONDARY, 2));
    if (axisID == 0)
    {
        double d1 = acos((pow(LENGTH_MAIN,2) + pow(c,2) - pow(LENGTH_SECONDARY,2)) / (2 * LENGTH_MAIN * c));
        double d2 = atan2(y,x);
        return ((d1 + d2) * radiansToDegrees);
        
    }
    else if (axisID == 1)
    {
        return (acos((pow(LENGTH_MAIN,2) + pow(LENGTH_SECONDARY,2) - pow(c,2)) / (2 * LENGTH_MAIN * LENGTH_SECONDARY)) * radiansToDegrees) - DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SECONDARY; //180 - might be wrong
    }
    else
    {
        return 0;
    }
}

void frc::Manipulator::moveToCoordinates(double x, double y) //Moves both axes to the angles necessary to reach the given coordinates
{
    manipSetPID(getAngleForCoordinates(x,y,0), 0);
    manipSetPID(getAngleForCoordinates(x,y,1), 1);
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