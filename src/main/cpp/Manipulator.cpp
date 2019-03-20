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
        //speed = getSpeed(.1, .4, currentDegrees, degrees, true);
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
        //speed = getSpeed(.1, .4, currentDegrees, degrees, false);
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
        //speed = getSpeed(.3, 1, currentDegrees, degrees, true);
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
    bool shoulderInPlace = false, elbowInPlace = false, wristInPlace = false;
    if (abs(currentRawShoulder - HOME_POSITION_SHOULDER_RAW) > 5)
    {
        if (abs(UNIVERSAL_SAFE_POSITION_ELBOW_RAW - getRawUnits(Elbow_Axis) > 20))
        {
            speed = getSpeed(.1, .6, currentRawElbow, UNIVERSAL_SAFE_POSITION_ELBOW_RAW, false);
            if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) > 20)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) < -20)
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
            speed = getSpeed(.1, .7, currentRawShoulder, HOME_POSITION_SHOULDER_RAW, true);
            if ((HOME_POSITION_SHOULDER_RAW - currentRawShoulder) > 5)
            {  
                Shoulder_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else if ((HOME_POSITION_SHOULDER_RAW - currentRawShoulder) < -5)
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
                shoulderInPlace = true;
            }
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
    }
    else if (abs(currentRawElbow - HOME_POSITION_ELBOW_RAW) > 5)
    {
        speed = getSpeed(.1, .8, currentRawElbow, HOME_POSITION_ELBOW_RAW, true);
        if ((HOME_POSITION_ELBOW_RAW - currentRawElbow) > 5)
        {  
            Elbow_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((HOME_POSITION_ELBOW_RAW - currentRawElbow) < -5)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
            elbowInPlace = true;
        }
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
    }
    else
    {
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        shoulderInPlace = true;
        elbowInPlace = true;
    }

    if (abs(currentRawWrist - HOME_POSITION_WRIST_RAW) > 5)
    {
        speed = getSpeed(.3, 1, currentRawWrist, HOME_POSITION_WRIST_RAW, true);
        if ((HOME_POSITION_WRIST_RAW - currentRawWrist) > 5)
        {  
            Wrist_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((HOME_POSITION_WRIST_RAW - currentRawWrist) < -5)
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, Off);
            wristInPlace = true; 
        }
    }
    else
    {
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        wristInPlace = true;
    }

    return (wristInPlace && elbowInPlace && shoulderInPlace);
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
    /*
        Example for Shoulder_Axis

        double resistance = (RESISTANCE1 * getRawUnits(Shoulder_Axis)) / (MAXIMUM_FEEDBACK - getRawUnits(Shoulder_Axis));
        double resistanceHome = (RESISTANCE1 * rawHome) / (MAXIMUM_FEEDBACK - rawHome);
        double degrees = (resistance - resistanceHome) / TO_DEGREES_SHOULDER;
        return abs(HOME_POSITION_SHOULDER_DEGREES + degrees);

    */
    if (axisID == Shoulder_Axis)
    {
        double degrees = (getRawUnits(Shoulder_Axis) - rawHome) / TO_DEGREES_SHOULDER;
        return abs(HOME_POSITION_SHOULDER_RAW + degrees); //-180 is offset   
    }
    else if (axisID == Elbow_Axis) //Needs changing
    {
        double degrees = (rawHome - getRawUnits(Elbow_Axis)) / TO_DEGREES_ELBOW;
        return abs(fmod(HOME_POSITION_ELBOW_RAW + degrees, 360));   
    }
    else if (axisID == Wrist_Axis) //Up = Increase in count
    {
        double degrees = (getRawUnits(Wrist_Axis) - rawHome) / TO_DEGREES_WRIST;
        return abs(fmod(HOME_POSITION_WRIST_RAW + degrees, 360));   
    }
    else
    {
        return 0;
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

    /*
    bool shoulderInCorrectPlace = abs(currentRawShoulder - rawShoulder) < 5;
    bool shoulderInSafePlace = abs(currentRawShoulder - optimizeAutomaticMovement(rawShoulder, rawElbow, Shoulder_Axis)) < 5;
    bool elbowInCorrectPlace = abs(currentRawElbow - rawElbow) < 5; 
    bool elbowInSafePlace = abs(currentRawElbow - optimizeAutomaticMovement(rawShoulder, rawElbow, Elbow_Axis)) < 5;
    bool wristInCorrectPlace = abs(currentRawWrist - rawWrist) < 5; 
    */
   
    bool shoulderInPlace = false, elbowInPlace = false, wristInPlace = false;
    
    /*
        if (!wristInPlace)
        {
            speed = getSpeed(.3, 1, currentRawWrist, rawWrist, false);
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
                wristInPlace = true; 
            }
        }  
        if (!elbowInSafePlace && !shoulderInSafePlace)
        {
            speed = getSpeed(.1, .6, currentRawElbow, UNIVERSAL_SAFE_POSITION_ELBOW_RAW, false);
            if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) > 20)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) < -20)
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, Off);
            }
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }  
        else if (elbowInSafePlace && !shoulderInSafePlace)
        {
            speed = getSpeed(.1, .7, currentRawShoulder, rawShoulder, true);
            if ((rawShoulder - currentRawShoulder) > 5)
            {  
                Shoulder_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else if ((rawShoulder - currentRawShoulder) < -5)
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
                shoulderInPlace = true;
            }
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
        else if (shoulderInSafePlace && !elbowInSafePlace)
        {
            speed = getSpeed(.1, .8, currentRawElbow, rawElbow, true);
            if ((rawElbow - currentRawElbow) > 5)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else if ((rawElbow- currentRawElbow) < -5)
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else
            {
                Elbow_Motor.Set(ControlMode::PercentOutput, Off);
                elbowInPlace = true;
            }
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }
        else
        {
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }

        return (wristInCorrectPlace && elbowInCorrectPlace && shoulderInCorrectPlace);

    */
    
    if (abs(currentRawShoulder - rawShoulder) > 5)
    {
        if (abs(UNIVERSAL_SAFE_POSITION_ELBOW_RAW - getRawUnits(Elbow_Axis)) > 20)
        {
           speed = getSpeed(.1, .6, currentRawElbow, UNIVERSAL_SAFE_POSITION_ELBOW_RAW, false);
            if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) > 20)
            {  
                Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
            }
            else if ((UNIVERSAL_SAFE_POSITION_ELBOW_RAW - currentRawElbow) < -20)
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
                speed = getSpeed(.16, .4, currentRawShoulder, rawShoulder, true);
            }
            else if (getRawUnits(Shoulder_Axis < 150))
            {
                speed = getSpeed(.1, .56, currentRawShoulder, rawShoulder, true);
            }
            else
            {
                speed = getSpeed(.1, .4, currentRawShoulder, rawShoulder, true);
            }
            
            if ((rawShoulder - currentRawShoulder) > 5)
            {  
                Shoulder_Motor.Set(ControlMode::PercentOutput, speed);
            }
            else if ((rawShoulder - currentRawShoulder) < -5)
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, -speed * .9);
            }
            else
            {
                Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
                shoulderInPlace = true;
            }
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
    }
    else if (abs(currentRawElbow - rawElbow) > 5)
    {
        speed = getSpeed(.1, .8, currentRawElbow, rawElbow, true);
        if ((rawElbow - currentRawElbow) > 5)
        {  
            Elbow_Motor.Set(ControlMode::PercentOutput, speed);
        }
        else if ((rawElbow- currentRawElbow) < -5)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, -speed);
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
            elbowInPlace = true;
        }
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
    }
    else
    {
        Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        shoulderInPlace = true;
        elbowInPlace = true;
    }

    if (abs(currentRawWrist - rawWrist) > 5)
    {
        speed = getSpeed(.3, 1, currentRawWrist, rawWrist, false);
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
            wristInPlace = true; 
        }
    }
    else
    {
        Wrist_Motor.Set(ControlMode::PercentOutput, Off);
        wristInPlace = true;
    }

    return (wristInPlace && elbowInPlace && shoulderInPlace);
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

double Blitz::Manipulator::optimizeAutomaticMovement(double desiredShoulder, double desiredElbow, int axisID)
{
    double currentShoulder = getRawUnits(Shoulder_Axis); //Replace with Degrees if possible
    double currentElbow = getRawUnits(Elbow_Axis); //Replace with Degrees if possible
    /*Rinse and Repeat as necessary (Likely many times)
    if (currentShoulder is within range A && currentElbow is within range A2)
    {
        if (axisID == Shoulder_Axis)
        {
            if (currentShoulder > desiredShoulder)
            {
                return val; //Higher value
            }
            else
            {
                return val2; //Lower value
            }
        }
        else if (axisID == Elbow_Axis)
        {
            if (currentElbow > desiredElbow)
            {
                return val3;
            }
            else
            {
                return val4;
            }
        }
    }
    else if (...)
    {
        ...
    }
    ...
    else
    {
        if (AxisID == Shoulder_Axis)
        {
            return trueSafe1;
        }
        else
        {
            return UNIVERSAL_SAFE_POSITION_ELBOW;
        }
    }
    */
   return 0;
}

double Blitz::Manipulator::getSpeed(double minSpeed, double maxSpeed, double currentPosition, double desiredPosition, bool isReversed)
{
    if(isReversed)
    {
        return -(1 - (1 / (abs((desiredPosition - currentPosition) * 0.03)+ 1))) * (maxSpeed - minSpeed) - minSpeed;
    }
    else
    {
        return (1 - (1 / (abs((desiredPosition - currentPosition) * 0.03)+ 1))) * (maxSpeed - minSpeed) + minSpeed;
    }
}

void Blitz::Manipulator::ResetPosition()
{
    if(LimitSwitch.Get())
    {
        ClawTalon.Set(ControlMode::PercentOutput, .7);
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
