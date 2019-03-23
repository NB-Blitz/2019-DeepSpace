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
        if((abs(speed)) > 0.1)
        {   
            Shoulder_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Shoulder_Motor.Set(ControlMode::PercentOutput, Off);
        }
    }
    else if (axisID == Elbow_Axis)
    {
        if ((abs(speed)) > 0.1)
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Elbow_Motor.Set(ControlMode::PercentOutput, Off);
        }
    }
    else if (axisID == Wrist_Axis)
    {
        if ((abs(speed)) > 0.1)
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, speed); 
        }
        else
        {
            Wrist_Motor.Set(ControlMode::PercentOutput, Off);
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
        speed = getSpeed(.1, 1, currentRawElbow, HOME_POSITION_ELBOW_RAW, true);
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

    /**/
    
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
                speed = getSpeed(.16, .60, currentRawShoulder, rawShoulder, true);
            }
            else if (getRawUnits(Shoulder_Axis) < 150)
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
        speed = getSpeed(.3, 1, currentRawWrist, rawWrist, true);
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

bool Blitz::Manipulator::ResetPosition()
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
    return LimitSwitch.Get();
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
