/*
  2/26/19
  - Coefficients are good as far as I can tell
  - Extraneous code temporarily removed (old code still saved in my system, so methods for the rest of the robot will be added later)
  - Currently testing moving to an angular position, which will quickly be followed by movement to coordinates of interest

  NOTE: Start in home position to guarantee accuracy with degrees
  Shoulder: 320 (about 37 minutes on a clock)
  Elbow: 40 
  Wrist: 90
*/

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
  Navx(SPI::Port::kMXP),
  Manipulator()
{

}

void Robot::RobotInit() 
{
  MecanumDrive.Initialize(&MecanumInput);
  MecanumDrive.SetMotorDirection(0, -1);
  MecanumDrive.SetMotorDirection(1, -1);
  MecanumDrive.SetMotorDirection(2, 1);
  MecanumDrive.SetMotorDirection(3, 1);

  LeftFrontMotor.ConfigOpenloopRamp(.4);
  LeftBackMotor.ConfigOpenloopRamp(.4);
  RightFrontMotor.ConfigOpenloopRamp(.4);
  RightBackMotor.ConfigOpenloopRamp(.4);

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
  Manipulator.InitializeArm();

  Navx.Reset();
  //Get Home Values
  /*
    As of 3/2/2019, these values are
    447,
    414,
    and 394 respectively
  */
  homeEncoderValueShoulder = Manipulator.getRawUnits(Shoulder_Axis);
  homeEncoderValueElbow = Manipulator.getRawUnits(Elbow_Axis);
  homeEncoderValueWrist = Manipulator.getRawUnits(Wrist_Axis);   
  
  //Display Home Values on SmartDashboard
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  frc::SmartDashboard::PutNumber("Home Encoder - Wrist", homeEncoderValueWrist);

  cout << "test1" << endl;

  while (IsOperatorControl() && IsEnabled()) 
  {
    //Receive Xbox input
    Xbox.update();
    Xbox2.update();
    LineTracker.Update();

    double XInput = -Xbox2.LeftX;
    double YInput = Xbox2.LeftY;
    double ZInput = -Xbox2.RightX;

    frc::SmartDashboard::PutNumber("XInput", XInput);
    frc::SmartDashboard::PutNumber("YInput", YInput);
    frc::SmartDashboard::PutNumber("ZInput", ZInput);

    if(!Xbox2.Xbox.GetRawButton(9))
    {
      Blitz::Models::MecanumInput FieldStuff = FieldControl.FieldControl(XInput, YInput, Navx.GetYaw());
      XInput = FieldStuff.XValue;
      YInput = FieldStuff.YValue;
    }

    axisShoulder = Xbox.LeftY;
    axisElbow = Xbox.RightY;
    axisWrist = Xbox.LeftX;
    
    /*
      Home = Catching Balls 

      For getting ball on low rocket 
      Shoulder Raw: 443
      Elbow Raw: 360
      Wrist Raw: 368

      For getting ball on medium rocket
      Shoulder Raw: 323
      Elbow Raw: 307
      Wrist Raw: 444

      For getting ball on high rocket
      Shoulder: 
      Elbow: 
      Wrist: 

      For getting hatch panel on low rocket & Receiving Hatch Panels
      Shoulder: 423
      Elbow: 289
      Wrist: 333

      For getting hatch panel on medium rocket
      Shoulder: 359
      Elbow: 333
      Wrist: 433

      For getting hatch panel on high rocket
      Shoulder: 
      Elbow: 
      Wrist: 

      For getting balls stuck out of robot frame
      Shoulder: 445
      Elbow: 296
      Wrist: 438
    */
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
        Manipulator.moveToRawCounts(323, 307, 444);
      }
      else
      {
        //Go to disc medium position on ship
        Manipulator.moveToRawCounts(359, 333, 433);
      }
    } 
    else if (Xbox.AButton)
    {
      if (ballToggle)
      {
        //Go to ball low position on ship
        Manipulator.moveToRawCounts(443, 360, 368);
      }
      else
      {
        //Go to disc low position on ship
        Manipulator.moveToRawCounts(423,289,333);
      }
    } 
    else if (Xbox.XButton)
    {
      if (ballToggle)
      {
        //Grab ball from the player station
        Manipulator.manipSetToHome();
      }
      else
      {
        //Grab disc from the player station
        Manipulator.moveToRawCounts(423,289,333);
      }
    }
    else if (Xbox.LeftBumper)
    {
      //Grab ball from robot frame
      Manipulator.moveToRawCounts(445,296,438);
    } 
    else if (Xbox.RightBumper)
    {
      if (ballToggle)
      {
        //Grab ball from the player station
        Manipulator.manipSetToHome();
      }
      else
      {
        //Grab disc from the player station
        Manipulator.moveToRawCounts(423,289,333);
      }
    }
    //Manual Code // Default Position
    else
    {
      if (manualToggle)
      {
        //Move joints manually
        Manipulator.manipSet(SPEED_MULTIPLIER_SHOULDER * axisShoulder, Shoulder_Axis, homeEncoderValueShoulder); 
        Manipulator.manipSet(SPEED_MULTIPLIER_ELBOW * axisElbow, Elbow_Axis, homeEncoderValueElbow);
        Manipulator.manipSet(SPEED_MULTIPLIER_WRIST * axisWrist, Wrist_Axis, homeEncoderValueWrist);
      }
      else
      {
        //Go to default position
        Manipulator.manipSetToHome();
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

    if(Xbox2.AButton)
    {
      AutoManager.DriveToBall(&MecanumInput);
    }
    
    //Receive raw input from potentiometers
    rawShoulder = Manipulator.getRawUnits(Shoulder_Axis);
    rawElbow = Manipulator.getRawUnits(Elbow_Axis);
    rawWrist = Manipulator.getRawUnits(Wrist_Axis);

    MecanumDrive.Run();

    //Record remaining information to SmartDashboard
    frc::SmartDashboard::PutNumber("Shoulder Pot Raw Current", rawShoulder);
    frc::SmartDashboard::PutNumber("Shoulder Pot Degrees", Manipulator.getDegrees(Shoulder_Axis, homeEncoderValueShoulder));
    frc::SmartDashboard::PutNumber("Elbow Pot Raw Current", rawElbow);
    frc::SmartDashboard::PutNumber("Elbow Pot Degrees", Manipulator.getDegrees(Elbow_Axis, homeEncoderValueElbow));
    frc::SmartDashboard::PutNumber("Wrist Pot Raw Current", rawWrist);
    frc::SmartDashboard::PutNumber("Wrist Pot Degrees", Manipulator.getDegrees(Wrist_Axis, homeEncoderValueWrist));
    frc::SmartDashboard::PutNumber("Shoulder's Axis", axisShoulder);
    frc::SmartDashboard::PutNumber("Elbow's Axis", axisElbow);
    frc::SmartDashboard::PutNumber("Wrist's Axis", axisWrist);
    frc::SmartDashboard::PutBoolean("IsHorizontal", Xbox.AButton);
    
    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    if(Xbox.RightTrigger > .1)
    {
      Manipulator.MoveManipulatorSpeed(Xbox.RightTrigger);
    }
    else if(Xbox.LeftTrigger > .1)
    {
      Manipulator.MoveManipulatorSpeed(-Xbox.LeftTrigger);
    }
    else
    {
      Manipulator.MoveManipulatorSpeed(0);
    }

    frc::Wait(0.005);
    }
  }
}


void Robot::Test() 
{
  while(IsTest() && IsEnabled())
  {
    double FGain = frc::SmartDashboard::GetNumber("FGain", 0.0);
    double PGain = frc::SmartDashboard::GetNumber("PGain", 0.0);
    double IGain = frc::SmartDashboard::GetNumber("IGain", 0.0);
    double DGain = frc::SmartDashboard::GetNumber("DGain", 0.0);

    int MotorNum = frc::SmartDashboard::GetNumber("MotorNum", 1);

    MecanumDrive.UsePID = !Xbox.LeftBumper;

    MecanumDrive.TuneF(1, FGain);
    MecanumDrive.TuneP(1, PGain);
    MecanumDrive.TuneI(1, IGain);
    MecanumDrive.TuneD(1, DGain);
    
    MecanumDrive.TuneF(2, FGain);
    MecanumDrive.TuneP(2, PGain);
    MecanumDrive.TuneI(2, IGain);
    MecanumDrive.TuneD(2, DGain);

    MecanumDrive.TuneF(3, FGain);
    MecanumDrive.TuneP(3, PGain);
    MecanumDrive.TuneI(3, IGain);
    MecanumDrive.TuneD(3, DGain);
    
    MecanumDrive.TuneF(4, FGain);
    MecanumDrive.TuneP(4, PGain);
    MecanumDrive.TuneI(4, IGain);
    MecanumDrive.TuneD(4, DGain);

    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    frc::Wait(0.005);
  }
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
