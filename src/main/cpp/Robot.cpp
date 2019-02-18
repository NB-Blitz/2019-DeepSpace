#include "Robot.h"

Robot::Robot()
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

    frc::Wait(0.005);
  }
}

void Robot::Test() 
{

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
