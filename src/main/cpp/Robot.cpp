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
  Xbox2(1),
  LineTracker(),
  Ultrasonics(0, 1),
  AutoManager(),
  Manip()
{

}

void Robot::RobotInit() 
{
  MecanumDrive.Initialize(&MecanumInput);
  MecanumDrive.SetMotorDirection(0, -1);
  MecanumDrive.SetMotorDirection(1, -1);
  MecanumDrive.SetMotorDirection(2, -1);
  MecanumDrive.SetMotorDirection(3, -1);


  frc::SmartDashboard::PutNumber("FGain", Blitz::DriveReference::MOTOR1_kF);
  frc::SmartDashboard::PutNumber("PGain", Blitz::DriveReference::MOTOR1_kP);
  frc::SmartDashboard::PutNumber("IGain", Blitz::DriveReference::MOTOR1_kI);
  frc::SmartDashboard::PutNumber("DGain", Blitz::DriveReference::MOTOR1_kD);
  frc::SmartDashboard::PutNumber("MotorNum", 1);
}

void Robot::Autonomous() 
{
  while(IsAutonomous() && IsEnabled())
  {
    AutoManager.DriveToBall(&MecanumInput);

    MecanumDrive.Run();
  }
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

    Xbox2.update();
    LineTracker.Update();
	

    double XInput = -Xbox2.LeftX;
    double YInput = Xbox2.LeftY;
    double ZInput = -Xbox2.RightX;

    if (Xbox2.RightStickButton)
    {
      XInput = LineTracker.GetDirections()[0];
      YInput = LineTracker.GetDirections()[1];
      ZInput = LineTracker.GetDirections()[2];
    }
    if (Ultrasonics.willCrash() && YInput > 0)
    {
      YInput = 0;
    }

    if(fabs(XInput) < .1)
    {
      XInput = 0;
    }

    if(fabs(YInput) < .1)
    {
      YInput = 0;
    }

    if(fabs(ZInput) < .1)
    {
      ZInput = 0;
    }

    MecanumInput.XValue = (XInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.YValue = (YInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.ZValue = (ZInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);

    if(Xbox2.LeftBumper)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }
    else if(Xbox2.RightBumper)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }

    if(Xbox2.LeftTrigger > .2)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox2.LeftTrigger;
    }
    else if(Xbox2.RightTrigger > .2)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox.RightTrigger;
    }

    if(Xbox2.AButton)
    {
      AutoManager.DriveToBall(&MecanumInput);
    }
    
    MecanumDrive.Run();

    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    if(Xbox2.RightBumper)
    {
      Manip.MoveManipulatorPosition(12);
    }
    else if(Xbox2.LeftBumper)
    {
      Manip.ResetPosition();
    }

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
        //Go to ball high position on ship
      }
      else
      {
        //Go to disc high position on ship
      }
      
    }
    else if (Xbox.BButton)
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
    else if (Xbox.AButton)
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
    else if (Xbox.XButton)
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
    else if (Xbox.LeftBumper)
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
    else if (Xbox.RightBumper)
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
        //Go to default position
      }
    }
    frc::SmartDashboard::PutBoolean("Manual Enabled", manualToggle);
    frc::SmartDashboard::PutBoolean("Ball Mode", ballToggle);
    frc::SmartDashboard::PutBoolean("Disc Mode", !ballToggle);

    Xbox2.update();
    LineTracker.Update();
	

    double XInput = -Xbox2.LeftX;
    double YInput = Xbox2.LeftY;
    double ZInput = -Xbox2.RightX;

    if (Xbox2.RightStickButton)
    {
      XInput = LineTracker.GetDirections()[0];
      YInput = LineTracker.GetDirections()[1];
      ZInput = LineTracker.GetDirections()[2];
    }
    if (Ultrasonics.willCrash() && YInput > 0)
    {
      YInput = 0;
    }

    if(fabs(XInput) < .1)
    {
      XInput = 0;
    }

    if(fabs(YInput) < .1)
    {
      YInput = 0;
    }

    if(fabs(ZInput) < .1)
    {
      ZInput = 0;
    }

    MecanumInput.XValue = (XInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.YValue = (YInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.ZValue = (ZInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);

    if(Xbox2.LeftBumper)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }
    else if(Xbox2.RightBumper)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }

    if(Xbox2.LeftTrigger > .2)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox2.LeftTrigger;
    }
    else if(Xbox2.RightTrigger > .2)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox.RightTrigger;
    }

    if(Xbox2.AButton)
    {
      AutoManager.DriveToBall(&MecanumInput);
    }
    
    MecanumDrive.Run();

    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    if(Xbox2.RightBumper)
    {
      Manip.MoveManipulatorPosition(12);
    }
    else if(Xbox2.LeftBumper)
    {
      Manip.ResetPosition();
    }

    frc::Wait(0.005);
  }

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
