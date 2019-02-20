#include "Robot.h"

Robot::Robot() :
  LeftFrontMotor(0),
  LeftBackMotor(1),
  RightFrontMotor(2),
  RightBackMotor(3),
  Motors(&LeftFrontMotor, &LeftBackMotor, &RightFrontMotor, &RightBackMotor),
  Logger(0),
  MecanumInput(),
  MecanumDrive(&Motors, &Logger),
  Xbox(0),
  LineTracker(),
  Ultrasonics(0, 1),
  AutoManager(),
  Manip()
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
  homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
  double rawWrist;
  double wristMin = 1024, wristMax = 0;
  /*homeEncoderValueShoulder = 0;
  homeEncoderValueElbow = 0;
  //homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
  //homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
  homeEncoderValueWrist = Manip.getRawUnits(Wrist_Axis);
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  frc::SmartDashboard::PutNumber("Home Encoder - Wrist", homeEncoderValueWrist);
  */
  while (IsOperatorControl() && IsEnabled()) 
  {
    Xbox.update();
    yAxisShoulder = Xbox.LeftY;
    yAxisElbow = Xbox.RightY;
    yAxisWrist = Xbox.LeftX;
    rawWrist = Manip.getRawUnits(Elbow_Axis);
    if (rawWrist > wristMax)
    {
      wristMax = rawWrist;
    }
    else if (rawWrist < wristMin)
    {
      wristMin = rawWrist;
    }
    frc::SmartDashboard::PutNumber("Elbow Pot Raw Max", wristMax);
    frc::SmartDashboard::PutNumber("Elbow Pot Raw Min", wristMin);
    frc::SmartDashboard::PutNumber("LeftY", yAxisShoulder);
    frc::SmartDashboard::PutNumber("RightY", yAxisElbow);
    frc::SmartDashboard::PutNumber("LeftX", yAxisWrist);
    frc::SmartDashboard::PutNumber("Raw Units - Elbow", Manip.getRawUnits(Elbow_Axis));
    frc::SmartDashboard::PutNumber("Degrees - Elbow", Manip.getDegrees(Elbow_Axis, homeEncoderValueElbow));
    Manip.manipSet(0.4*yAxisShoulder, Shoulder_Axis, 0);
    Manip.manipSet(0.4*yAxisElbow, Elbow_Axis, homeEncoderValueElbow);
    Manip.manipSet(0.4*yAxisWrist, Wrist_Axis, 0);

  /*

    Manip.manipSet(0.4 * Xbox.LeftY, Wrist_Axis, homeEncoderValueWrist);
    frc::SmartDashboard::PutNumber("Y-Axis", Xbox.LeftY);
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
    yAxisShoulder = Xbox.LeftY;
    yAxisElbow = Xbox.RightY;
    frc::SmartDashboard::PutNumber("LeftY", yAxisShoulder);
    frc::SmartDashboard::PutNumber("RightY", yAxisElbow);

    if (Xbox.AButton)
    {
      Manip.moveToCoordinates(10, -3.5, homeEncoderValueShoulder, homeEncoderValueElbow); //Low
    }
    else if (Xbox.BButton)
    {
      Manip.moveToCoordinates(10, 3, homeEncoderValueShoulder, homeEncoderValueElbow); //Mid
    }
    else if (Xbox.YButton)
    {
      Manip.moveToCoordinates(10, 8, homeEncoderValueShoulder, homeEncoderValueElbow); //High
    }
    else
    {
      //Shoulder Axis
      if (Xbox.XButton)
      {
        Manip.manipSetToDegrees(180, Shoulder_Axis, homeEncoderValueShoulder);
      }
      else 
      {
        
        if (yAxisShoulder > 0.1 || yAxisShoulder < -0.1)
        {
          Manip.manipSet(yAxisShoulder, Shoulder_Axis, homeEncoderValueShoulder);
        }
        else
        {
          Manip.manipSet(Off, Shoulder_Axis, homeEncoderValueShoulder);
        }          
      }

      //Elbow Axis
      if (Xbox.LeftBumper)
      {
        Manip.manipSetToDegrees(180, Elbow_Axis, homeEncoderValueElbow);
      }
      else 
      {
        if (yAxisElbow > 0.1 || yAxisElbow < -0.1)
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
  */
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
    Xbox.update();
    //Toggle between Ball and Disc
    if (Xbox.Xbox.GetRawButton(7) && !isStartDown)
    {
      ballToggle = !ballToggle;
    }
    isStartDown = Xbox.Xbox.GetRawButton(7);

    //Toggle between allowed manual and disabled manual (fully automatic)
    if (Xbox.Xbox.GetRawButton(6) && !isBackDown)
    { 
      manualToggle = !manualToggle;
    }
    isBackDown = Xbox.Xbox.GetRawButton(6);

    //Hard-coded positions
    if (Xbox.YButton)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Go to ball high position on ship
      }
      else
      {
        //Manip.moveToCoordinates();
        //Go to disc high position on ship
      }
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Straight_Up);
    }
    else if (Xbox.BButton)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Go to ball medium position on ship
      }
      else
      {
        //Manip.moveToCoordinates();
        //Go to disc medium position on ship
      }
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Straight_Up);
    } 
    else if (Xbox.AButton)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Go to ball low position on ship
      }
      else
      {
        //Manip.moveToCoordinates();
        //Go to disc low position on ship
      }
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Straight_Up);
    } 
    else if (Xbox.XButton)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Go to ball habitat position
      }
      else
      {
        //Manip.moveToCoordinates();
        //Go to disc habitat position
      }
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Straight_Up);
    }
    else if (Xbox.LeftBumper)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Grab ball from the ground
      }
      else
      {
        //Manip.moveToCoordinates();
        //Grab disc from the ground
      }
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Parallel);
    } 
    else if (Xbox.RightBumper)
    {
      if (ballToggle)
      {
        //Manip.moveToCoordinates();
        //Grab ball from the player station
      }
      else
      {
        //Manip.moveToCoordinates();
        //Grab disc from the player station
      }

    }
    //Manual Code // Default Position
    else
    {
      if (manualToggle)
      {
        leftYAxis = Xbox.LeftY;
        rightYAxis = Xbox.RightY;
        
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
        //Manip.moveToCoordinates();
        //Go to default position
      }
      //Moves wrist to parallel with the ground
      Manip.moveToXDegreesBelowParallel(homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist, Straight_Up);
    }
  frc::SmartDashboard::PutBoolean("Manual Enabled", manualToggle);
  frc::SmartDashboard::PutBoolean("Ball Mode", ballToggle);
  frc::SmartDashboard::PutBoolean("Disc Mode", !ballToggle);
  frc::Wait(0.005);
  }

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
