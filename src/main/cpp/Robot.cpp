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
  
}

void Robot::OperatorControl() 
{
  Navx.Reset();
  while (IsOperatorControl() && IsEnabled()) 
  {
    Xbox.update();

    double XInput = Xbox.RightX;
    double YInput = -Xbox.RightY;
    double ZInput = Xbox.LeftX;

    FieldControl(XInput, YInput);
    XInput = newX;
    YInput = newY;

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
  //Field Oriented Control

  void Robot::FieldControl(double x, double y)
  {
    float pi = 3.1415926; 
    double angle = 0;
    double angleR = ((Navx.GetYaw() + 180) * pi) / 180.0;
    double angleJ = atan2(y, x) + pi;
    double m = sqrt(pow(x,2) + pow(y,2)); //Delcaring r (magnitude); 
    frc::SmartDashboard::PutNumber("Angle",angleR);

    //For Quadrant 4
    if(angleJ > 90 && angleJ < 270)
    {
      angle = pi + angleJ;
    }
    else if(angleJ > 270)
    {
      angle = (2 *pi) + angleJ;
    }

    else
    {
      angle = angleJ;
    }
    
    angle += angleR;

    newX = m * cos(angle);
    newY = m * sin(angle);
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
