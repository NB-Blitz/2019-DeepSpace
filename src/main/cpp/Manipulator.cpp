#include "Manipulator.hpp"
#include "math.h"

Blitz::Manipulator::Manipulator() :
    Shoulder_Motor(4),
    Elbow_Motor(5),    
    Wrist_Motor(6),
    LimitSwitch(0),
    ClawTalon(7),
    PositionCounter(1)
{

}

void Blitz::Manipulator::manipSet(double speed, int axisID, double rawHome) //Moves Manipulator in accordance to the joystick 
{
    if (axisID == Shoulder_Axis)
    {
        bool cancel1 = ((getDegrees(Shoulder_Axis, rawHome) >= MAX_RANGE_SHOULDER) && speed > 0);
        bool cancel2 = ((getDegrees(Shoulder_Axis, rawHome) <= MIN_RANGE_SHOULDER) && speed < 0);
        if(/*!(cancel1 || cancel2) && */(abs(speed)) > 0.1)
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
        if (/*!(cancel1 || cancel2) && */(abs(speed)) > 0.1)
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
        if (/*!(cancel1 || cancel2) && */(abs(speed)) > 0.1)
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

void Blitz::Manipulator::manipSetToDegrees(double degrees, int axisID, double rawHome) //Go to a set angular position
{
    double currentDegrees;
    currentDegrees = getDegrees(axisID, rawHome);
    if (axisID == Shoulder_Axis) //Prevents movement to unsafe areas
    {
        double speed = -(1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1))) * .3 - .1;
        if ((degrees - currentDegrees) > 1.5)
        {  
            manipSet(speed * 1.5, Shoulder_Axis, rawHome);
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
        double speed = (1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1))) * .3 + .1;
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
    else if (axisID == Wrist_Axis)
    {
        double speed = -(1 - (1 / (abs((degrees-currentDegrees) * 0.03)+ 1))) * .7 - .3;
        if ((degrees - currentDegrees) > 1.5)
        {
            manipSet(speed, Wrist_Axis, rawHome);
        }
        else if ((degrees - currentDegrees) < -1.5)
        {
            manipSet(-speed, Wrist_Axis, rawHome);
        }
        else
        {
            manipSet(Off, Wrist_Axis, rawHome);
        }
    }
}

bool Blitz::Manipulator::manipSetToHome()
{
    /*
    As of 3/2/2019, these values are
    447,
    414,
    and 394 respectively
  */
    double currentRawShoulder = getRawUnits(Shoulder_Axis);
    double currentRawElbow = getRawUnits(Elbow_Axis);
    double currentRawWrist = getRawUnits(Wrist_Axis);
    double speed;
    if (abs(currentRawWrist - 394) > 5)
    {
        speed = -(1 - (1 / (abs((394 - currentRawWrist) * 0.03)+ 1))) * .7 - .3;
        if ((394 - currentRawWrist) > 5)
        {  
            Wrist_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((394 - currentRawWrist) < -5)
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        return false;
    }
    else if (abs(currentRawShoulder - 447) > 5)
    {
        if (abs(318 - getRawUnits(Elbow_Axis) > 20))
        {
            speed = (1 - (1 / (abs((318 - currentRawElbow) * 0.03)+ 1))) * .5 + .1;
            if ((318 - currentRawElbow) > 20)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else if ((318 - currentRawElbow) < -20)
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, Off);
            }
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }
        else
        {
            speed = -(1 - (1 / (abs((447 - currentRawShoulder) * 0.03)+ 1))) * .6 - .1;
            if ((447 - currentRawShoulder) > 5)
            {  
                Shoulder_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else if ((447 - currentRawShoulder) < -5)
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
            }
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        return false;
    }
    else if (abs(currentRawElbow - 414) > 5)
    {
        speed = -(1 - (1 / (abs((414 - currentRawElbow) * 0.03)+ 1))) * .7 - .1;
        if ((414 - currentRawElbow) > 5)
        {  
            Elbow_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((414 - currentRawElbow) < -5)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        return false;
    }
    else
    {
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        return true;
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
        return abs(fmod(HOME_POSITION_WRIST + degrees, 360));   
    }
    else
    {
        return 0;
    }   
}

void Blitz::Manipulator::resetDegrees(int axisID) //Resets the encoder to 0 raw counts
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

bool Blitz::Manipulator::moveToRawCounts(double rawShoulder, double rawElbow, double rawWrist)
{
    double currentRawShoulder = getRawUnits(Shoulder_Axis);
    double currentRawElbow = getRawUnits(Elbow_Axis);
    double currentRawWrist = getRawUnits(Wrist_Axis);
    double speed;
    if (abs(currentRawWrist - rawWrist) > 5)
    {
        speed = -(1 - (1 / (abs((rawWrist - currentRawWrist) * 0.03)+ 1))) * .7 - .3;
        if ((rawWrist - currentRawWrist) > 5)
        {  
            Wrist_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((rawWrist - currentRawWrist) < -5)
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
    }
    else if (abs(currentRawShoulder - rawShoulder) > 5)
    {
        if (abs(318 - getRawUnits(Elbow_Axis)) > 20 && currentRawShoulder > 150)
        {
            speed = (1 - (1 / (abs((318 - currentRawElbow) * 0.03)+ 1))) * .4 + .3;
            if ((318 - currentRawElbow) > 20)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else if ((318 - currentRawElbow) < -20)
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, Off);
            }
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }
        else
        {
            if (getRawUnits(Shoulder_Axis) > 320)
            {
                speed = -(1 - (1 / (abs((rawShoulder - currentRawShoulder) * 0.03)+ 1))) * .3 - .2;
            }
            else if(getRawUnits(Shoulder_Axis) < 150)
            {
                speed = -(1 - (1 / (abs((rawShoulder - currentRawShoulder) * 0.03)+ 1))) * .6 - .1;
            }
            else
            {
                speed = -(1 - (1 / (abs((rawShoulder - currentRawShoulder) * 0.03)+ 1))) * .4 - .1;
            }
            if ((rawShoulder - currentRawShoulder) > 5)
            {  
                Shoulder_Motor.Set(ControlMode::PercentOutput, speed * .7);
            }
            else if ((rawShoulder - currentRawShoulder) < -5)
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, -speed * .6);
            }
            else
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
            }
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
    }
    else if (abs(currentRawElbow - rawElbow) > 5)
    {
        speed = -(1 - (1 / (abs((rawElbow - currentRawElbow) * 0.03)+ 1))) * .3 - .1;
        if ((rawElbow - currentRawElbow) > 5)
        {  
            Elbow_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((rawElbow - currentRawElbow) < -5)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
    }
    else
    {
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        return true;
    }
    return false;
}

void Blitz::Manipulator::moveToAngles(double shoulderAngle, double elbowAngle, double wristAngle, double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist) //Moves both axes to the angles necessary to reach the given coordinates
{
    if (abs(getDegrees(Wrist_Axis, rawHomeWrist) - wristAngle) >= 5)
    {
        manipSetToDegrees(wristAngle, Wrist_Axis, rawHomeWrist);
    }
    else if (abs(getDegrees(Shoulder_Axis, rawHomeShoulder) - shoulderAngle >= 5))
    {
        
        if (abs(getDegrees(Elbow_Axis, rawHomeElbow) - 60) >= 5)
        {
            manipSetToDegrees(60, Elbow_Axis, rawHomeElbow);
        }
        else
        {
            manipSetToDegrees(shoulderAngle, Shoulder_Axis, rawHomeShoulder);
        }   
    }
    else
    {
        manipSetToDegrees(elbowAngle, Elbow_Axis, rawHomeElbow);
    }
}

void Blitz::Manipulator::moveToCoordinates(double x, double y, double rawHomeShoulder, double rawHomeElbow) //Moves both axes to the angles necessary to reach the given coordinates
{
    if (abs(getDegrees(Elbow_Axis, rawHomeShoulder) - 75) >= 5 && abs(getDegrees(Shoulder_Axis, rawHomeShoulder) - getAngleForCoordinates(x,y,Shoulder_Axis) >= 5))
    {
        manipSetToDegrees(75, Elbow_Axis, rawHomeElbow);
    }
    else if (abs(getDegrees(Shoulder_Axis, rawHomeShoulder) - getAngleForCoordinates(x,y,Shoulder_Axis) >= 5))
    {
        manipSetToDegrees(getAngleForCoordinates(x,y,Shoulder_Axis), Shoulder_Axis, rawHomeShoulder);
    }
    else
    {
        manipSetToDegrees(getAngleForCoordinates(x,y,Elbow_Axis), Elbow_Axis, rawHomeElbow);
    }  
}

bool Blitz::Manipulator::isPossible(double x, double y) //Returns whether the arm can physically and legally move to coordinates
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
void Blitz::Manipulator::moveToXDegreesBelowParallel(double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist, double x) //Moves wrist to X degrees below parallel
{
    double degrees1 = getDegrees(Shoulder_Axis, rawHomeShoulder);
    double degrees2 = getDegrees(Elbow_Axis, rawHomeElbow);
    manipSetToDegrees(360 - degrees1 - degrees2 + x - MIN_RANGE_WRIST, Wrist_Axis, rawHomeWrist);
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
void Blitz::Manipulator::InitializeArm()
{
    Shoulder_Motor.ConfigOpenloopRamp(1);
    Elbow_Motor.ConfigOpenloopRamp(1);
    Wrist_Motor.ConfigOpenloopRamp(.3);
}