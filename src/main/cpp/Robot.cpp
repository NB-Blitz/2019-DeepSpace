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
  Navx(SPI::Port::kMXP)
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
  while (IsOperatorControl() && IsEnabled()) 
  {
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

    if(Xbox.LeftBumper)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }
    else if(Xbox.RightBumper)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * .75;
    }

    if(Xbox.LeftTrigger > .2)
    {
      MecanumInput.XValue = Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox.LeftTrigger;
    }
    else if(Xbox.RightTrigger > .2)
    {
      MecanumInput.XValue = -Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND * Xbox.LeftTrigger;
    }

    if(Xbox.AButton)
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
//Robot.setWorking(true);
//bool workNormally = true;
#endif
