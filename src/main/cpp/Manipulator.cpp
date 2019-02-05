#include "Manipulator.hpp"
#include "math.h"

frc::Manipulator::Manipulator() :
    Main_Axis(6), 
    Main_Axis_Limit_Switch(0)//,
    //Secondary_Axis(8),
    //Secondary_Axis_Limit_Switch(1)//Placeholder ID
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
    /*
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
        double encoderCounts = (3649 / 180) * degrees;
        if (!(cancel1 || cancel2))
        {
            
            Main_Axis.Set(ControlMode::Position, encoderCounts); 
        }
        frc::SmartDashboard::PutBoolean("Canceler Logic 1 (Max)", cancel1);
        frc::SmartDashboard::PutBoolean("Canceler Logic 2 (Limit)", cancel2);
    }
    /*
    else if (axisID == 1)
    {
        bool cancel1 = (degrees >= MAX_RANGE_SECONDARY);
        bool cancel2 = (isLimit(1) && (degrees < getDegrees(1)));
        if (!(cancel1 || cancel2))
        {
            double encoderCounts = degrees * TO_DEGREES_SECONDARY;
            Secondary_Axis.Set(ControlMode::Position, encoderCounts); 
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
    /*
    else if (axisID == 1)
    {
        return !Secondary_Axis_Limit_Switch.Get();   
    }
    */
    return true;
}

double frc::Manipulator::getDegrees(int axisID) //Returns degrees from an encoder converting from raw counts
{
    if (axisID == 0)
    {
        double degrees =  Main_Axis.GetSelectedSensorPosition(0) / TO_DEGREES_MAIN;
        return abs(fmod(degrees, 360));   
    }/*
    else if (axisID == 1)
    {
        double degrees =  Secondary_Axis.GetSelectedSensorPosition(0) / TO_DEGREES_SECONDARY;
        return abs(fmod(degrees, 360));   
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
        //Secondary_Axis.SetSelectedSensorPosition(0, 0, 0);
    }
   
}
//Positive = down, negative = up

void frc::Manipulator::moveTo(double degrees, int axisID) //Old method to move the manipulator to a set degree without PID -> can be used but I hope to replace with manipSetPID
{
    double currDegrees = getDegrees(axisID);
    if (axisID == 0)
    {
        if (currDegrees > degrees + DEAD_ZONE_MAIN)
        {
            manipSet(-0.2, axisID);
        }
        else if (currDegrees < degrees - DEAD_ZONE_MAIN)
        {
            manipSet(0.2, axisID);
        }
    }
    else if (axisID == 1)
    {
        /*
        if (currDegrees > degrees + DEAD_ZONE_SECONDARY)
        {
            manipSet(-0.2, 1);
        }
        else if (currDegrees < degrees - DEAD_ZONE_SECONDARY)
        {
            manipSet(0.2, 1);
        }
        */
    }
}

void frc::Manipulator::resetToEncoder(int axisID) //Resets the manipulator to the position where the limit switch is active, then resets the encoder
{
    
    if (!isLimit(axisID))
    {
        manipSet(-0.3, axisID);    
    }
    else
    {
        manipSet(0, axisID);
        resetDegrees(axisID);
    }
}

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
    /*
    if (firstTime)
    {
        Secondary_Axis.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
        Secondary_Axis.SetSensorPhase(true);
        Secondary_Axis.ConfigNominalOutputForward(0, 30);
        Secondary_Axis.ConfigNominalOutputReverse(0, 30);
        Secondary_Axis.ConfigPeakOutputForward(1, 30);
        Secondary_Axis.ConfigPeakOutputReverse(-1, 30);
    }   
    Secondary_Axis.Config_kP(0, PID_P_MAIN, 30);
    Secondary_Axis.Config_kI(0, PID_I_MAIN, 30);
    Secondary_Axis.Config_kD(0, PID_D_MAIN, 30);
    Secondary_Axis.Config_kF(0, PID_F_MAIN, 30);
    */
}

double frc::Manipulator::getP(int axisID) //Returns the Proportional Multiplier of PID
{
    if (axisID == 0)
    {
        return PID_P_MAIN;
    }
    /*
    else if (axisID == 1)
    {
        return PID_P_SECONDARY;
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
    /*
    else if (axisID == 1)
    {
        return PID_I_SECONDARY;
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
    /*
    else if (axisID == 1)
    {
        return PID_D_SECONDARY;
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
    /*
    else if (axisID == 1)
    {
        return PID_F_SECONDARY;
    }
    */
    else
    {
        return -1;
    }
}
/*
void frc::Manipulator::updatePIDCoefficients() //Supposed to update PID values from SmartDashboard - Currently not tested to be functional
{
    double prePMain = PID_P_MAIN;
    double preIMain = PID_I_MAIN;
    double preDMain = PID_D_MAIN;
    double preFMain = PID_F_MAIN;
    PID_P_MAIN = frc::SmartDashboard::GetNumber("P-Main", 0);
    PID_I_MAIN = frc::SmartDashboard::GetNumber("I-Main", 0);
    PID_D_MAIN = frc::SmartDashboard::GetNumber("D-Main", 0);
    PID_F_MAIN = frc::SmartDashboard::GetNumber("F-Main", 0);
    if ((prePMain != PID_P_MAIN) || (preIMain != PID_I_MAIN) || (preDMain != PID_D_MAIN) || (preFMain != PID_F_MAIN))
    {
        initializePID(false);
    }
}
*/
/*
double frc::Manipulator::getAngleForCoordinates(double x, double y, int axisID) //Returns the angle of an axis necessary for reaching the given coordinates
{
    double pi = 3.14159265358979323846;
    double radiansToDegrees = (180 / pi);
    double c = sqrt(pow(LENGTH_MAIN, 2) + pow(LENGTH_SECONDARY, 2));
    if (axisID == 0)
    {
        double d1 = acos((pow(LENGTH_MAIN,2) + pow(c,2) - pow(LENGTH_SECONDARY,2)) / (2 * LENGTH_MAIN * c));
        double d2 = atan2(y,x);
        return (d1 + d2) * radiansToDegrees;
        
    }
    else if (axisID == 1)
    {
        return 180 - (acos((pow(LENGTH_MAIN,2) + pow(LENGTH_SECONDARY,2) - pow(c,2)) / (2 * LENGTH_MAIN * LENGTH_SECONDARY)) * radiansToDegrees);
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
*/