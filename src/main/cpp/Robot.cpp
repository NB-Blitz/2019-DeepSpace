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
  Manipulator(),
  climber()
{

}

void Robot::RobotInit() 
{
  Navx.Reset();
  climber.StartCompressor();
  Manipulator.InitializeArm();
  MecanumDrive.Initialize(&MecanumInput);

  MecanumDrive.SetMotorDirection(0, -1);
  MecanumDrive.SetMotorDirection(1, -1);
  MecanumDrive.SetMotorDirection(2, 1);
  MecanumDrive.SetMotorDirection(3, 1);

  LeftFrontMotor.ConfigOpenloopRamp(DRIVETRAIN_RAMP_TIME);
  LeftBackMotor.ConfigOpenloopRamp(DRIVETRAIN_RAMP_TIME);
  RightFrontMotor.ConfigOpenloopRamp(DRIVETRAIN_RAMP_TIME);
  RightBackMotor.ConfigOpenloopRamp(DRIVETRAIN_RAMP_TIME);

  frc::SmartDashboard::PutNumber("FGain", Blitz::DriveReference::MOTOR1_kF);
  frc::SmartDashboard::PutNumber("PGain", Blitz::DriveReference::MOTOR1_kP);
  frc::SmartDashboard::PutNumber("IGain", Blitz::DriveReference::MOTOR1_kI);
  frc::SmartDashboard::PutNumber("DGain", Blitz::DriveReference::MOTOR1_kD);
}

void Robot::Autonomous() 
{
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

  while (IsAutonomous() && IsEnabled()) 
  {
    RunRobot();

    frc::Wait(0.005);
  }
}

void Robot::OperatorControl() 
{
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

  while (IsOperatorControl() && IsEnabled()) 
  {
    RunRobot();

    frc::Wait(0.005);
  }
}

void Robot::RunRobot()
{
     //Receive Xbox input
    Xbox.update();
    Xbox2.update();
    LineTracker.Update();

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

      Ball Top position
      Shoulder: 121
      Elbow: 82
      Wrist: 437
    */


    //Toggle between allowed manual and disabled manual (fully automatic)
    if (Xbox.RightStickButton && !isRightStickDown)
    { 
      manualToggle = !manualToggle;
    }
    isRightStickDown = Xbox.RightStickButton;

    if(Xbox.RightBumper && !isTriggerPressed)
    {
      CurrentElbowPosition = Manipulator.getRawUnits(Elbow_Axis);
      CurrentWristPosition = Manipulator.getRawUnits(Wrist_Axis);
      CurrentShoulderPosition = Manipulator.getRawUnits(Shoulder_Axis);
    }
    else if(Xbox.RightBumper)
    {
      inPosition = Manipulator.moveToRawCounts(CurrentShoulderPosition, CurrentElbowPosition - 10, CurrentWristPosition);
    }
    //Hard-coded positions
    else if (Xbox.YButton)
    {
        //Go to ball medium position on ship
        inPosition = Manipulator.moveToRawCounts(323, 307, 444);
    }
    else if (Xbox.BButton)
    {
      //Go to disc medium position on ship
      inPosition = Manipulator.moveToRawCounts(359, 333, 433);
    } 
    else if (Xbox.AButton)
    {
      inPosition = Manipulator.moveToRawCounts(423,289,333);
    } 
    else if (Xbox.XButton)
    {
      //Go to ball low position on ship
      inPosition = Manipulator.moveToRawCounts(443, 360, 368);
    }
    else if (Xbox.LeftBumper)
    {
      //Grab ball from robot frame
      inPosition = Manipulator.moveToRawCounts(445,296,438);
    }
    //Manual Code/Default Position
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
        inPosition = Manipulator.manipSetToHome();
      }
    }
    isTriggerPressed = Xbox.RightBumper;

    //Move Claw
    if(Xbox.RightTrigger > JOYSTICK_DEADBAND)
    {
      Manipulator.ResetPosition();
    }
    else if(Xbox.LeftTrigger > JOYSTICK_DEADBAND)
    {
      Manipulator.MoveManipulatorSpeed(-Xbox.LeftTrigger);
    }
    else
    {
      Manipulator.MoveManipulatorSpeed(0);
    }
    
    //Driving Code
    double XInput = -Xbox2.LeftX;
    double YInput = Xbox2.LeftY;
    double ZInput = -Xbox2.RightX * 0.8;

    //Deadbands
    if(fabs(XInput) < JOYSTICK_DEADBAND)
    {
      XInput = 0;
    }

    if(fabs(YInput) < JOYSTICK_DEADBAND)
    {
      YInput = 0;
    }

    if(fabs(ZInput) < JOYSTICK_DEADBAND)
    {
      ZInput = 0;
    }

    //Drive Modes
    if(Xbox2.YButton)
    {
      XInput = 0;
      ZInput *= .8;
    }
    else if(Xbox2.RightStickButton)
    {
      XInput *= .5;
      YInput *= .5;
      ZInput *= .5;
    } 
    else if((Ultrasonics.willCrash() && YInput < 0) && !Xbox2.RightStickButton) //Ultrasonic stop code
    {
      YInput = 0;
    }
    else if(Xbox2.LeftStickButton) //Field Oriented Control (Normally Disabled)
    {
      Blitz::Models::MecanumInput FieldInput = FieldControl.FieldControl(XInput, YInput, Navx.GetYaw());
      XInput = FieldInput.XValue;
      YInput = FieldInput.YValue;
    }
    else if(Xbox2.LeftBumper) //Left Strafe
    {
      XInput = STRAFE_SPEED;
    }
    else if(Xbox2.RightBumper) //Right Strafe
    {
       XInput = -STRAFE_SPEED;
    }

    climber.SetBackSolenoid(Xbox2.YButton);

    //Populating MecanumInput
    MecanumInput.XValue = (XInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.YValue = (YInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.ZValue = (ZInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);

    //Runs BallTracking
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
    frc::SmartDashboard::PutBoolean("In Position", inPosition);
    
    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

}

void Robot::Test()
{
  
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
