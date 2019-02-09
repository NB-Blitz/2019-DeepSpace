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
double rotateDegreesMain, rotateDegreesSecondary;//, rotateDegreesWrist;
bool isMain = true;

void Robot::RobotInit() 
{
}

void Robot::RobotPeriodic() 
{
  
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {
  rotateDegreesMain = 90;
  rotateDegreesSecondary = 30;//, rotateDegreesWrist;
  Manip.initializePID(true); //PID is initialized
  Manip.resetDegrees(0);
}
void Robot::TeleopPeriodic() {
  std::cout << "Beginning of RobotPeriodic: " << rotateDegreesMain << std::endl;
  double yAxis = Blitz_Joy.getAxis(1); 
  isMain = !Blitz_Joy.getButton(6);
  if (Blitz_Joy.getButton(1)) //Resets Encoder value to 0 
  {
    if (isMain)
    {
      Manip.resetDegrees(0);
    }
    else
    {
      Manip.resetDegrees(1);
    }
  }
  else if (Blitz_Joy.getButton(2)) //Sets Manipulator to 45 degrees using PID
  {
    if (isMain)
    {
      Manip.manipSetPID(135, 0);
    }
    else
    {
      Manip.manipSetPID(135, 1);
    }
  }
  else if (Blitz_Joy.getButton(3)) //Sets Manipulator to 90 degrees using PID
  {
    if (isMain)
    {
      Manip.manipSetPID(180, 0);
    }
    else
    {
      Manip.manipSetPID(180, 1);
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
      Manip.resetToEncoder(1);
    }
  }
  else if (Blitz_Joy.getButton(5))
  {
    Manip.moveToCoordinates(5,5);
    //Manip.moveToParallel(5,5);
  }
  else //Manual control of Manipulator with joystick
  {
    if (isMain)
    {
      
      //Manip.manipSetPID(rotateDegreesSecondary, 1); // * Holds axes not in use in its previous position
      if (yAxis > 0.1 || yAxis < -0.1)
      {
        Manip.manipSet(yAxis, 0); 
        rotateDegreesMain = Manip.getDegrees(0);
        std::cout << "ON" << std::endl;
      }
      else
      {
        Manip.manipSetPID(rotateDegreesMain, 0); //*
        std::cout << "OFF" << std::endl;
      }
      
    }
    else
    { 
      rotateDegreesSecondary = Manip.getDegrees(1); 
      //Manip.manipSetPID(rotateDegreesMain, 0);  //Holds axes not in use in its previous position
      Manip.manipSet(yAxis, 1);
    }
    
  }

  frc::SmartDashboard::PutNumber("TestEnc", Manip.Main_Axis.GetSelectedSensorPosition(0));
  frc::SmartDashboard::PutNumber("Y-Axis", yAxis);
  frc::SmartDashboard::PutBoolean("IsResetButton", Blitz_Joy.getButton(1));
  frc::SmartDashboard::PutBoolean("IsLimitSwitch-Main", Manip.isLimit(0));
  frc::SmartDashboard::PutNumber("Rotate Degrees - Main", rotateDegreesMain);
  frc::SmartDashboard::PutNumber("Rotate Degrees - Secondary", rotateDegreesSecondary);
  frc::SmartDashboard::PutNumber("Real Degrees - Main", Manip.getDegrees(0));
  frc::SmartDashboard::PutNumber("Real Degrees - Secondary", Manip.getDegrees(1));
  //frc::SmartDashboard::PutNumber("Rotate Degrees - Wrist", rotateDegreesWrist);
  frc::SmartDashboard::PutBoolean("IsLimitSwitch-Secondary", Manip.isLimit(1));
  frc::SmartDashboard::PutNumber("P-Main", Manip.getP(0));
  frc::SmartDashboard::PutNumber("I-Main", Manip.getI(0));
  frc::SmartDashboard::PutNumber("D-Main", Manip.getD(0));
  frc::SmartDashboard::PutNumber("F-Main", Manip.getF(0));
  frc::SmartDashboard::PutNumber("P-Secondary", Manip.getP(1));
  frc::SmartDashboard::PutNumber("I-Secondary", Manip.getI(1));
  frc::SmartDashboard::PutNumber("D-Secondary", Manip.getD(1));
  frc::SmartDashboard::PutNumber("F-Secondary", Manip.getF(1));
  //frc::SmartDashboard::PutNumber("P-Wrist", Manip.getP(2));
  //frc::SmartDashboard::PutNumber("I-Wrist", Manip.getI(2));
  //frc::SmartDashboard::PutNumber("D-Wrist", Manip.getD(2));
  //frc::SmartDashboard::PutNumber("F-Wrist", Manip.getF(2));
  frc::SmartDashboard::PutBoolean("isMain", isMain);
  std::cout << "End of RobotPeriodic: " << rotateDegreesMain << std::endl;
  frc::Wait(0.005);
}
void Robot::TestPeriodic() {
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif