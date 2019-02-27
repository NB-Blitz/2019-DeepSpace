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
  Navx.Reset();
  //Get Home Values
  homeEncoderValueShoulder = Manipulator.getRawUnits(Shoulder_Axis);
  homeEncoderValueElbow = Manipulator.getRawUnits(Elbow_Axis);
  homeEncoderValueWrist = Manipulator.getRawUnits(Wrist_Axis);   
  
  //Display Home Values on SmartDashboard
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  frc::SmartDashboard::PutNumber("Home Encoder - Wrist", homeEncoderValueWrist);

  while (IsOperatorControl() && IsEnabled()) 
  {
    //Receive Xbox input
    Xbox.update();
    LineTracker.Update();
	

    double XInput = -Xbox.LeftX;
    double YInput = Xbox.LeftY;
    double ZInput = -Xbox.RightX;

    if(!Xbox.Xbox.GetRawButton(9))
    {
      Blitz::Models::MecanumInput FieldStuff = FieldControl.FieldControl(XInput, YInput, Navx.GetYaw());
      XInput = FieldStuff.XValue;
      YInput = FieldStuff.YValue;
    }

    if (Xbox.RightStickButton)
    axisShoulder = Xbox.LeftY;
    axisElbow = Xbox.RightY;
    axisWrist = Xbox.LeftX;
    
    if (Xbox.AButton)
    {
      //Move all joints so that the arm sticks out horizontally
      Manipulator.manipSetToDegrees(270, Shoulder_Axis, homeEncoderValueShoulder);
      Manipulator.manipSetToDegrees(180, Elbow_Axis, homeEncoderValueElbow);
      Manipulator.manipSetToDegrees(180, Wrist_Axis, homeEncoderValueWrist);
    }
    else
    {
      //Move joints manually
      Manipulator.manipSet(SPEED_MULTIPLIER_SHOULDER * axisShoulder, Shoulder_Axis, homeEncoderValueShoulder); //Gearbox is having issues
      Manipulator.manipSet(SPEED_MULTIPLIER_ELBOW * axisElbow, Elbow_Axis, homeEncoderValueElbow);
      Manipulator.manipSet(SPEED_MULTIPLIER_WRIST * axisWrist, Wrist_Axis, homeEncoderValueWrist);
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

    if(Xbox.LeftBumper)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }
    else if(Xbox.RightBumper)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }

    if(Xbox.AButton)
    {
      AutoManager.DriveToBall(&MecanumInput);
    }
    
    //Receive raw input from potentiometers
    rawShoulder = Manipulator.getRawUnits(Shoulder_Axis);
    rawElbow = Manipulator.getRawUnits(Elbow_Axis);
    rawWrist = Manipulator.getRawUnits(Wrist_Axis);

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
    
    //Delay
    frc::SmartDashboard::PutNumber("FrontLeftJoyStick", MecanumDrive.GetMotorOutput(1));
    frc::SmartDashboard::PutNumber("BackLeftJoyStick", MecanumDrive.GetMotorOutput(2));
    frc::SmartDashboard::PutNumber("FrontRightJoyStick", MecanumDrive.GetMotorOutput(3));
    frc::SmartDashboard::PutNumber("BackRightJoyStick", MecanumDrive.GetMotorOutput(4));

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    if(Xbox.RightBumper)
    {
      Manipulator.MoveManipulatorPosition(12);
    }
    else if(Xbox.LeftBumper)
    {
      Manipulator.ResetPosition();
    }

    frc::Wait(0.005);
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
