/*
  2/12/19
  Code is currently functional, but possibly not optimized
  Speed is currently hardcoded to .25 to protect prototype, but the commented speed function is better given a better prototype
  Three test cases (low, medium, high) all maintain x-axis pretty well (within .2 inches)

  TODO
  - Go away from limit switches and start from a known
  * Limits work within normal functions, but break during planned movement (coordinates)
  - Add isPossible to rest of code (add hard limit for distance of 30 inches)
  - (If time, work on paths)


  Note:
  Code without limits CANNOT use DEGREES_BETWEEN constants also!
*/

#include "Robot.h"
#include "Blitz_Joystick.hpp"
#include "Manipulator.hpp"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


//Test Variable -> true for limit switches, false for pre-set known
bool areLimits = false;

//if the previous variable is true, these variables are relevant
bool initialReset = true;

//if the previous variable is false, these variables are relevant
double homeEncoderValueShoulder, homeEncoderValueElbow;

frc::Manipulator Manip;
frc::Blitz_Joystick Blitz_Joy;
double rotateDegreesMain, rotateDegreesSecondary;//, rotateDegreesWrist;
double yAxisShoulder, yAxisElbow;

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() 
{
  Manip.initializePID(true); //PID is initialized
  if (!areLimits) //It is assumed that the robot is set in the home position (both axes are at about 90 degrees)
  {
    homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
    homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
    frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
    frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueShoulder);
  }
}
//Highly recommended to reset to encoder both axes before starting
void Robot::TeleopPeriodic() {
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
        //Main Axis
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

        //Secondary Axis
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
      //Main Axis
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

      //Secondary Axis
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
  }
  
  frc::SmartDashboard::PutNumber("Shoulder Motor Degrees", Manip.getDegrees(Shoulder_Axis));
  frc::SmartDashboard::PutNumber("Shoulder Motor Raw Encoder Units", Manip.getRawUnits(Shoulder_Axis));
  frc::SmartDashboard::PutNumber("Elbow Motor Degrees", Manip.getDegrees(Elbow_Axis));
  frc::SmartDashboard::PutNumber("Elbow Motor Raw Encoder Units", Manip.getRawUnits(Elbow_Axis));

  frc::Wait(0.005);
}
void Robot::TestPeriodic() {
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif