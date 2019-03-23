/*
  3/19/19
  This code has but does not utlize optimization code
  This is meant to test the "unStickManipulator" code, nothing more
  Everything else should be identical to the dev branch, besides some minor pretested housekeeping fixes
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
  ArmController(0),
  DriveController(1),
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
    ArmController.update();
    DriveController.update();
    LineTracker.Update();

    axisShoulder = ArmController.LeftY;
    axisElbow = -ArmController.RightX;
    axisWrist = ArmController.LeftX;
    
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

      For getting hatch panel on low rocket & Receiving Hatch Panels
      Shoulder: 423
      Elbow: 289
      Wrist: 333

      For getting hatch panel on medium rocket
      Shoulder: 359
      Elbow: 333
      Wrist: 433

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
    if (ArmController.RightStickButton && !isRightStickDown)
    { 
      manualToggle = !manualToggle;
    }
    isRightStickDown = ArmController.RightStickButton;

    if(DriveController.AButton && !isTriggerPressed)
    {
      CurrentElbowPosition = Manipulator.getRawUnits(Elbow_Axis);
      CurrentWristPosition = Manipulator.getRawUnits(Wrist_Axis);
      CurrentShoulderPosition = Manipulator.getRawUnits(Shoulder_Axis);
    }
    //Hard-coded positions
    else if (ArmController.YButton)
    {
      //Go to ball medium position on ship
      inPosition = Manipulator.moveToRawCounts(323, 307, 444);
    }
    else if (ArmController.BButton)
    {
      //Go to disc medium position on ship
      inPosition = Manipulator.moveToRawCounts(359, 333, 433);
    } 
    else if (ArmController.AButton)
    {
      inPosition = Manipulator.moveToRawCounts(423,280,333);
    } 
    else if (ArmController.XButton)
    {
      //Go to ball low position on ship
      inPosition = Manipulator.moveToRawCounts(443, 356, 368);
    }
    else if (ArmController.LeftBumper)
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
    isTriggerPressed = DriveController.AButton;

    //Move Claw
    if(ArmController.RightTrigger > JOYSTICK_DEADBAND)
    {
      Manipulator.MoveManipulatorSpeed(1);
    }
    else if(ArmController.LeftTrigger > JOYSTICK_DEADBAND)
    {
      Manipulator.MoveManipulatorSpeed(-1);
    }
    else
    {
      Manipulator.MoveManipulatorSpeed(0);
    }
    
    //Driving Code
    double XInput = -DriveController.LeftX;
    double YInput = DriveController.LeftY;
    double ZInput = -DriveController.RightX * 0.8;

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
    if(DriveController.RightBumper)
    {
      XInput = 0;
      ZInput *= .8;
    }
    else if(DriveController.RightStickButton)
    {
      XInput *= .5;
      YInput *= .5;
      ZInput *= .5;
    } 
    else if(DriveController.LeftStickButton) //Field Oriented Control (Normally Disabled)
    {
      Blitz::Models::MecanumInput FieldInput = FieldControl.FieldControl(XInput, YInput, Navx.GetYaw());
      XInput = FieldInput.XValue;
      YInput = FieldInput.YValue;
    }
    else if(DriveController.LeftBumper) //Left Strafe
    {
      XInput = STRAFE_SPEED;
    }
    else if(DriveController.RightBumper) //Right Strafe
    {
       XInput = -STRAFE_SPEED;
    }

    climber.SetBackSolenoid(DriveController.RightBumper);

    
  
    //Populating MecanumInput
    MecanumInput.XValue = (XInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.YValue = (YInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    MecanumInput.ZValue = (ZInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);

    //Receive raw input from potentiometers
    rawShoulder = Manipulator.getRawUnits(Shoulder_Axis);
    rawElbow = Manipulator.getRawUnits(Elbow_Axis);
    rawWrist = Manipulator.getRawUnits(Wrist_Axis);

    //Overrides drive and arm control to automatically back away from where the hatch panel was placed
    if(DriveController.AButton)
    {
      inPosition = unStickManipulator(CurrentShoulderPosition, CurrentElbowPosition, CurrentWristPosition);
    }

    MecanumDrive.Run();

    //Record remaining information to SmartDashboard
    frc::SmartDashboard::PutNumber("Elbow Start Position", CurrentElbowPosition);
    frc::SmartDashboard::PutNumber("Shoulder Pot Raw Current", rawShoulder);
    frc::SmartDashboard::PutNumber("Elbow Pot Raw Current", rawElbow);
    frc::SmartDashboard::PutNumber("Wrist Pot Raw Current", rawWrist);
    frc::SmartDashboard::PutNumber("Shoulder's Axis", axisShoulder);
    frc::SmartDashboard::PutNumber("Elbow's Axis", axisElbow);
    frc::SmartDashboard::PutNumber("Wrist's Axis", axisWrist);
    frc::SmartDashboard::PutBoolean("IsHorizontal", ArmController.AButton);
    frc::SmartDashboard::PutBoolean("In Position", inPosition);
    frc::SmartDashboard::PutBoolean("Manual Mode", manualToggle);
    
    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

}

bool Robot::unStickManipulator(double CurrentShoulderPosition, double CurrentElbowPosition, double CurrentWristPosition)
{
  Manipulator.ResetPosition();
  if (Manipulator.moveToRawCounts(CurrentShoulderPosition, CurrentElbowPosition - 10, CurrentWristPosition))
  {
    //Move backward slowly
    MecanumInput.XValue = 0;
    MecanumInput.YValue = .15 * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
    MecanumInput.ZValue = 0;
    return true;
  }

  return false;
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

    MecanumDrive.UsePID = !ArmController.LeftBumper;

    ArmController.update();

    MecanumInput.YValue = ArmController.LeftY;

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

    MecanumDrive.Run();

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
