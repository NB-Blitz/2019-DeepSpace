/*
  2/16/2019 4:20 P.M.
  
  TO-DO:
  -Work toward removing Limit Switches from code completely and replace with Potentiometers
  -In Robot.cpp, remove areLimits = true code
  -Add remaining potentiometers to code and find coefficients
*/

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
  double rawWrist;
  double wristMin = 1024, wristMax = 0;
  homeEncoderValueShoulder = 0;
  homeEncoderValueElbow = 0;
  //homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
  //homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
  homeEncoderValueWrist = Manip.getRawUnits(Wrist_Axis);
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  frc::SmartDashboard::PutNumber("Home Encoder - Wrist", homeEncoderValueWrist);
  while (IsOperatorControl() && IsEnabled()) 
  {
    Manip.manipSet(0.4 * Blitz_Joy.getAxis(Y_Axis, Shoulder_Axis), Wrist_Axis, homeEncoderValueWrist);
    frc::SmartDashboard::PutNumber("Y-Axis", Blitz_Joy.getAxis(Y_Axis, Shoulder_Axis));
    rawWrist = Manip.getRawUnits(Wrist_Axis);
    if (rawWrist > wristMax)
    {
      wristMax = rawWrist;
    }
    else if (rawWrist < wristMin)
    {
      wristMin = rawWrist;
    }

    frc::SmartDashboard::PutNumber("Wrist Pot Raw", rawWrist);
    frc::SmartDashboard::PutNumber("Wrist Pot Raw Max", wristMax);
    frc::SmartDashboard::PutNumber("Wrist Pot Raw Min", wristMin);
    frc::SmartDashboard::PutNumber("Wrist Pot Degrees", Manip.getDegrees(Wrist_Axis, homeEncoderValueWrist));
    yAxisShoulder = Blitz_Joy.getAxis(Y_Axis, Shoulder_Joystick); 
    yAxisElbow = Blitz_Joy.getAxis(Y_Axis, Elbow_Joystick);

    if (Blitz_Joy.getButton(3, Shoulder_Joystick))
    {
      Manip.moveToCoordinates(10, -3.5, homeEncoderValueShoulder, homeEncoderValueElbow); //Low
    }
    else if (Blitz_Joy.getButton(4, Shoulder_Joystick))
    {
      Manip.moveToCoordinates(10, 3, homeEncoderValueShoulder, homeEncoderValueElbow); //Mid
    }
    else if (Blitz_Joy.getButton(5, Shoulder_Joystick))
    {
      Manip.moveToCoordinates(10, 8, homeEncoderValueShoulder, homeEncoderValueElbow); //High
    }
    else
    {
      //Shoulder Axis
      if (Blitz_Joy.getButton(2, Shoulder_Joystick))
      {
        Manip.manipSetToDegrees(180, Shoulder_Axis, homeEncoderValueShoulder);
      }
      else 
      {
        
        if (yAxisShoulder > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisShoulder < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
        {
          Manip.manipSet(yAxisShoulder, Shoulder_Axis, homeEncoderValueShoulder);
        }
        else
        {
          Manip.manipSet(Off, Shoulder_Axis, homeEncoderValueShoulder);
        }          
      }

      //Elbow Axis
      if (Blitz_Joy.getButton(2, Elbow_Joystick))
      {
        Manip.manipSetToDegrees(180, Elbow_Axis, homeEncoderValueElbow);
      }
      else 
      {
        if (yAxisElbow > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisElbow < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
        {
          Manip.manipSet(yAxisElbow, Elbow_Axis, homeEncoderValueElbow);
        }
        else
        {
          Manip.manipSet(Off, Elbow_Axis, homeEncoderValueElbow);
        }
      }
    }
  frc::SmartDashboard::PutNumber("Shoulder Motor Degrees", Manip.getDegrees(Shoulder_Axis, homeEncoderValueShoulder));
  frc::SmartDashboard::PutNumber("Shoulder Motor Raw Encoder Units", Manip.getRawUnits(Shoulder_Axis));
  frc::SmartDashboard::PutNumber("Elbow Motor Degrees", Manip.getDegrees(Elbow_Axis, homeEncoderValueElbow));
  frc::SmartDashboard::PutNumber("Elbow Motor Raw Encoder Units", Manip.getRawUnits(Elbow_Axis));
  frc::Wait(0.005);
  }
}
/*
  XBox Controller Button Map
  ==========================
  Start Button -> Toggle between ball and disc mode
  Back Button -> Toggle between manual and automatic default
  Y Button -> High Position on ship
  B Button -> Medium Position on ship
  A Button -> Low Position on ship
  X Button -> Habitat Position
  Left Bumper -> Grab from ground
  Right Bumper -> Grab from player station
  Left Axis -> If manual enabled, move shoulder
  Right Axis -> If manual enabled, move elbow
*/
void Robot::Test() //This is test code using the xBox Controller (for some reason, it won't run on Test mode, but the toggles work correctly when put in OperatorControl())
{
  bool ballToggle = true; //if true, then balls - if false, then discs
  bool manualToggle = false; //if true, then manual is allowed - if false, then manual is disabled
  bool isStartDown = false;
  bool isBackDown = false;
  double leftYAxis;
  double rightYAxis;
  homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
  homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  while(IsTest() && IsEnabled())
  {
    //Toggle between Ball and Disc
    if (Blitz_Joy.getXBoxButton(XBox_Start_Button) && !isStartDown)
    {
      ballToggle = !ballToggle;
    }
    isStartDown = Blitz_Joy.getXBoxButton(XBox_Start_Button);

    //Toggle between allowed manual and disabled manual (fully automatic)
    if (Blitz_Joy.getXBoxButton(XBox_Back_Button) && !isBackDown)
    { 
      manualToggle = !manualToggle;
    }
    isBackDown = Blitz_Joy.getXBoxButton(XBox_Back_Button);

    //Hard-coded positions
    if (Blitz_Joy.getXBoxButton(XBox_Y_Button))
    {
      if (ballToggle)
      {
        //Go to ball high position on ship
      }
      else
      {
        //Go to disc high position on ship
      }
      
    }
    else if (Blitz_Joy.getXBoxButton(XBox_B_Button))
    {
      if (ballToggle)
      {
        //Go to ball medium position on ship
      }
      else
      {
        //Go to disc medium position on ship
      }
    } 
    else if (Blitz_Joy.getXBoxButton(XBox_A_Button))
    {
      if (ballToggle)
      {
        //Go to ball low position on ship
      }
      else
      {
        //Go to disc low position on ship
      }
    } 
    else if (Blitz_Joy.getXBoxButton(XBox_X_Button))
    {
      if (ballToggle)
      {
        //Go to ball habitat position
      }
      else
      {
        //Go to disc habitat position
      }
    }
    else if (Blitz_Joy.getXBoxButton(XBox_Left_Bumper))
    {
      if (ballToggle)
      {
        //Grab ball from the ground
      }
      else
      {
        //Grab disc from the ground
      }
    } 
    else if (Blitz_Joy.getXBoxButton(XBox_Right_Bumper))
    {
      if (ballToggle)
      {
        //Grab ball from the player station
      }
      else
      {
        //Grab disc from the player station
      }
    }
    //Manual Code // Default Position
    else
    {
      if (manualToggle)
      {
        leftYAxis = Blitz_Joy.getXBoxAxis(XBox_Left_Y_Axis);
        rightYAxis = Blitz_Joy.getXBoxAxis(XBox_Right_Y_Axis);
        
        //Manual Shoulder Code
        if (abs(leftYAxis) > 0.1)
        {
          Manip.manipSet(leftYAxis * 0.4, Shoulder_Axis, homeEncoderValueShoulder);
        }
        else
        {
          Manip.manipSet(Off, Shoulder_Axis, homeEncoderValueShoulder);
        }
        
        //Manual Elbow Code
        if (abs(rightYAxis) > 0.1)
        {
          Manip.manipSet(rightYAxis * 0.4, Elbow_Axis, homeEncoderValueElbow);
        }
        else
        {
          Manip.manipSet(Off, Elbow_Axis, homeEncoderValueElbow);
        }

      }
      else
      {
        //Go to default position
      }
    }
  frc::SmartDashboard::PutBoolean("Manual Enabled", manualToggle);
  frc::SmartDashboard::PutBoolean("Ball Mode", ballToggle);
  frc::SmartDashboard::PutBoolean("Disc Mode", !ballToggle);
  }

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
