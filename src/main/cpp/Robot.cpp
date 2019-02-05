/*
  IMPORTANT NOTE: Some features using PID and all features using the second motor are commented out
*/
#include "Robot.h"
#include "Blitz_Joystick.hpp"
#include "Manipulator.hpp"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


frc::Manipulator Manip;
frc::Blitz_Joystick Blitz_Joy;
double rotateDegrees;
bool isMain = true;

void Robot::RobotInit() 
{
  Manip.initializePID(true); //PID is initialized
}

void Robot::RobotPeriodic() 
{
  double yAxis = Blitz_Joy.getAxis(1); 
  rotateDegrees = Manip.getDegrees(0); //Purely for SmartDashboard
  //Manip.updatePIDCoefficients();
  if (Blitz_Joy.getButton(1)) //Resets Encoder value to 0 
  {
    if (isMain)
    {
      Manip.resetDegrees(0);
    }
    else
    {
      //Manip.resetDegrees(1);
    }
  }
  else if (Blitz_Joy.getButton(2)) //Sets Manipulator to 45 degrees using PID
  {
    if (isMain)
    {
      Manip.manipSetPID(45, 0);
    }
    else
    {
      //Manip.manipSetPID(45, 1);
    }
  }
  else if (Blitz_Joy.getButton(3)) //Sets Manipulator to 90 degrees using PID
  {
    if (isMain)
    {
      Manip.manipSetPID(90, 0);
    }
    else
    {
      //Manip.manipSetPID(90, 1);
    }
    
  }
  else if (Blitz_Joy.getButton(4)) //Moves Manipulator until it hits its limit switch, then resets its encoder
  {
    if (isMain)
    {
      Manip.resetToEncoder(0);
    }
    else
    {
      //Manip.resetToEncoder(1);
    }
  }
  else //Manual control of Manipulator with joystick
  {
    if (isMain)
    {
      Manip.manipSet(yAxis, 0);
    }
    else
    { 
      //Manip.manipSet(yAxis, 1);
    }
  }

  frc::SmartDashboard::PutNumber("Y-Axis", yAxis);
  frc::SmartDashboard::PutBoolean("IsResetButton", Blitz_Joy.getButton(1));
  frc::SmartDashboard::PutBoolean("IsLimitSwitch-Main", Manip.isLimit(0));
  frc::SmartDashboard::PutNumber("Rotate Degrees", rotateDegrees);
  //frc::SmartDashboard::PutBoolean("IsLimitSwitch-Secondary", Manip.isLimit(1));
  frc::SmartDashboard::PutNumber("P-Main", Manip.getP(0));
  frc::SmartDashboard::PutNumber("I-Main", Manip.getI(0));
  frc::SmartDashboard::PutNumber("D-Main", Manip.getD(0));
  frc::SmartDashboard::PutNumber("F-Main", Manip.getF(0));
  //frc::SmartDashboard::PutNumber("P-Secondary", Manip.getP(1));
  //frc::SmartDashboard::PutNumber("I-Secondary", Manip.getI(1));
  //frc::SmartDashboard::PutNumber("D-Secondary", Manip.getD(1));
  //frc::SmartDashboard::PutNumber("F-Secondary", Manip.getF(1));
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
