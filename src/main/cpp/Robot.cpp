#include "Robot.h"

Robot::Robot() :
  Xbox(0),
  Claw()
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
  while (IsOperatorControl() && IsEnabled()) 
  {
    Xbox.update();

    if(Xbox.XButton)
    {
      Claw.ResetPosition();
    }
    else if(Xbox.AButton)
    {
      Claw.MoveManipulatorPosition(18);
    }
    else if(Xbox.BButton)
    {
      Claw.MoveManipulatorPosition(5);
    }
    else
    {
      Claw.MoveManipulatorSpeed(Xbox.LeftY);
    }

    frc::Wait(0.005);
  }
}

void Robot::Test() 
{

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
