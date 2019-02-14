#include "Robot.h"

Robot::Robot() :
  Manip(),
  Blitz_Joy()
{

}

void Robot::RobotInit() 
{
  
}

void Robot::Autonomous() 
{
  
}

void Robot::OperatorControl() 
{
  //Manip.initializePID(true); //PID is initialized
  if (!areLimits) //It is assumed that the robot is set in the home position (both axes are at about 90 degrees)
  {
    homeEncoderValueShoulder = 0;
    homeEncoderValueElbow = 0;
    //Only necessary if the encoder's zero is not where the rest position is
    homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
    homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
    frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
    frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  }
  while (IsOperatorControl() && IsEnabled()) 
  {
    
    if (areLimits) //Code for presence of limit switches
    {
      if (initialReset)
      {
        if (Manip.resetToEncoder(Shoulder_Axis) && Manip.resetToEncoder(Elbow_Axis))
        {
          initialReset = false;
        }
      }
      else
      {
        yAxisShoulder = Blitz_Joy.getAxis(Y_Axis, Shoulder_Joystick); 
        yAxisElbow = Blitz_Joy.getAxis(Y_Axis, Elbow_Joystick);

        if (Blitz_Joy.getButton(3, Shoulder_Joystick))
        {
          Manip.moveToCoordinates(10, -3.5); //Low
        }
        else if (Blitz_Joy.getButton(4, Shoulder_Joystick))
        {
          Manip.moveToCoordinates(10, 3); //Mid
        }
        else if (Blitz_Joy.getButton(5, Shoulder_Joystick))
        {
          Manip.moveToCoordinates(10, 8); //High
        }
        else
        {
          //Shoulder Axis
          if (Blitz_Joy.getButton(1, Shoulder_Joystick)) //Reset To Encoder
          {
            Manip.resetToEncoder(Shoulder_Axis);
          }
          else if (Blitz_Joy.getButton(2, Shoulder_Joystick))
          {
            Manip.manipSetToDegrees(180, Shoulder_Axis);
          }
          else
          {
            if (yAxisShoulder > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisShoulder < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
            {
              Manip.manipSet(yAxisShoulder, Shoulder_Axis);
            }
            else
            {
              Manip.manipSet(Off, Shoulder_Axis);
            }
          }

          //Elbow Axis
          if (Blitz_Joy.getButton(1, Elbow_Joystick))
          {
            Manip.resetToEncoder(Elbow_Axis);
          }
          else if (Blitz_Joy.getButton(2, Elbow_Joystick))
          {
            Manip.manipSetToDegrees(180, Elbow_Axis);
          }
          else
          {
            if (yAxisElbow > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisElbow < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
            {
              Manip.manipSet(yAxisElbow, Elbow_Axis);
            }
            else
            {
              Manip.manipSet(Off, Elbow_Axis);
            }
          }
        }
      }
    frc::SmartDashboard::PutNumber("Shoulder Motor Degrees", Manip.getDegrees(Shoulder_Axis));
    frc::SmartDashboard::PutNumber("Shoulder Motor Raw Encoder Units", Manip.getRawUnits(Shoulder_Axis));
    frc::SmartDashboard::PutNumber("Shoulder Motor Degrees (If Limits existed)", Manip.getDegrees(Shoulder_Axis));
    frc::SmartDashboard::PutNumber("Elbow Motor Degrees", Manip.getDegrees(Elbow_Axis));
    frc::SmartDashboard::PutNumber("Elbow Motor Raw Encoder Units", Manip.getRawUnits(Elbow_Axis));
    frc::SmartDashboard::PutNumber("Elbow Motor Degrees (If Limits existed)", Manip.getDegrees(Elbow_Axis));
    }
    else //Code for lack of limit switches
    {
      yAxisShoulder = Blitz_Joy.getAxis(Y_Axis, Shoulder_Joystick); 
      yAxisElbow = Blitz_Joy.getAxis(Y_Axis, Elbow_Joystick);

      if (Blitz_Joy.getButton(3, Shoulder_Joystick))
      {
        Manip.moveToCoordinates(10, -3.5, false, homeEncoderValueShoulder, homeEncoderValueElbow); //Low
      }
      else if (Blitz_Joy.getButton(4, Shoulder_Joystick))
      {
        Manip.moveToCoordinates(10, 3, false, homeEncoderValueShoulder, homeEncoderValueElbow); //Mid
      }
      else if (Blitz_Joy.getButton(5, Shoulder_Joystick))
      {
        Manip.moveToCoordinates(10, 8, false, homeEncoderValueShoulder, homeEncoderValueElbow); //High
      }
      else
      {
        //Shoulder Axis
        if (Blitz_Joy.getButton(2, Shoulder_Joystick))
        {
          Manip.manipSetToDegrees(180, Shoulder_Axis, false, homeEncoderValueShoulder);
        }
        else
        {
          if (yAxisShoulder > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisShoulder < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
          {
            Manip.manipSet(yAxisShoulder, Shoulder_Axis, false, homeEncoderValueShoulder);
          }
          else
          {
            Manip.manipSet(Off, Shoulder_Axis, false, homeEncoderValueShoulder);
          }
        }

        //Elbow Axis
        if (Blitz_Joy.getButton(2, Elbow_Joystick))
        {
          Manip.manipSetToDegrees(180, Elbow_Axis, false, homeEncoderValueElbow);
        }
        else
        {
          if (yAxisElbow > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisElbow < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
          {
            Manip.manipSet(yAxisElbow, Elbow_Axis, false, homeEncoderValueElbow);
          }
          else
          {
            Manip.manipSet(Off, Elbow_Axis, false, homeEncoderValueElbow);
          }
        }
      }
    frc::SmartDashboard::PutNumber("Shoulder Motor Degrees", Manip.getDegrees(Shoulder_Axis, homeEncoderValueShoulder));
    frc::SmartDashboard::PutNumber("Shoulder Motor Degrees (If Limits existed)", Manip.getDegrees(Shoulder_Axis));
    frc::SmartDashboard::PutNumber("Shoulder Motor Raw Encoder Units", Manip.getRawUnits(Shoulder_Axis));
    frc::SmartDashboard::PutNumber("Elbow Motor Degrees", Manip.getDegrees(Elbow_Axis, homeEncoderValueElbow));
    frc::SmartDashboard::PutNumber("Elbow Motor Degrees (If Limits existed)", Manip.getDegrees(Elbow_Axis));
    frc::SmartDashboard::PutNumber("Elbow Motor Raw Encoder Units", Manip.getRawUnits(Elbow_Axis));
    }
  frc::Wait(0.005);
  }
}

void Robot::Test() 
{
  while(IsTest() && IsEnabled())
  {
    frc::Wait(0.005);
  }
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
