#include "Robot.h"

Robot::Robot() :
  LeftFrontMotor(1),
  LeftBackMotor(2),
  RightFrontMotor(3),
  RightBackMotor(4),
  Motors(&LeftFrontMotor, &LeftBackMotor, &RightFrontMotor, &RightBackMotor),
  Logger(0),
  MecanumInput(),
  MecanumDrive(&Motors, &Logger),
  Xbox(0),
  Navx(SPI::Port::kMXP)
{

}

void Robot::RobotInit() 
{
  MecanumDrive.Initialize(&MecanumInput);

  frc::SmartDashboard::PutNumber("FGain", Blitz::DriveReference::MOTOR1_kF);
  frc::SmartDashboard::PutNumber("PGain", Blitz::DriveReference::MOTOR1_kP);
  frc::SmartDashboard::PutNumber("IGain", Blitz::DriveReference::MOTOR1_kI);
  frc::SmartDashboard::PutNumber("DGain", Blitz::DriveReference::MOTOR1_kD);
  frc::SmartDashboard::PutNumber("MotorNum", 1);
}

void Robot::Autonomous() 
{
  double Speed = 0.3;

  bool direction = true; //True starts on the left, false on the right

  double stageOne = 1.5;
  double stageTwo = 5.25;
  double stageThree = 5.5;
  double stageFour = 5.6;
  double stageFive = 10.5;
  double stageSix = 10.6;
  double stageSeven = 15.5;

  double turnAngle = 180;

  frc::Timer seconds; 
  seconds.Start();

  Navx.ZeroYaw();

  while(IsAutonomous() && IsEnabled())
  {

    if(seconds.Get() <= stageOne)
    {
      //Go forwards
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }

    else if (seconds.Get() <= stageTwo)
    {
      if (direction == true)
      {
        //Diagonal right
        MecanumInput.XValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.ZValue = 0;
      }
      
      else
      {
        //Diagonal left
        MecanumInput.XValue = -Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.ZValue = 0;
      }
    }

    else if (seconds.Get() <= stageThree)
    {
      //Go forwards
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }
   
    else if(seconds.Get() <= stageFour)
    {
      //Place the hatch panel/cover on the hatch during this time
      seconds.Stop();
      MecanumInput.XValue = 0;
      MecanumInput.YValue = 0;
      MecanumInput.ZValue = 0;
      seconds.Start();
    }

    else if (seconds.Get() <= stageFive)
    {
      if (direction == true)
      {
        //Go straight left
        MecanumInput.XValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }

      else
      {
        //Go straight right
        MecanumInput.XValue = -Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }
    }

    else if (seconds.Get() <= stageSix)
    {
      //Turn 180 degrees
      seconds.Stop();
      frc::SmartDashboard::PutNumber("Angle", Navx.GetYaw());
      if (Navx.GetYaw() >= 0 && Navx.GetYaw() < turnAngle)
      {
        MecanumInput.XValue = 0;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = (-(turnAngle - Navx.GetYaw()) * Speed * (1/turnAngle) + -Speed / 2) * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      }
      else
      {
        MecanumInput.XValue = 0;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }
      seconds.Start();
    }
    
    else if (seconds.Get() <= stageSeven)
    {
      //Move forward
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }
    
    else 
    {
      //Grab the hatch from the wall during this stopped time. 
      seconds.Stop();
      MecanumInput.XValue = 0;
      MecanumInput.YValue = 0;
      MecanumInput.ZValue = 0;
    }
    MecanumDrive.Run();

    frc::Wait(0.015);

  }
}


void Robot::OperatorControl() 
{
  while (IsOperatorControl() && IsEnabled()) 
  {
    Xbox.update();

    double XInput = -Xbox.RightX;
    double YInput = Xbox.RightY;
    double ZInput = -Xbox.LeftX;

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
