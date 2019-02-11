/*
  2/11/19
  Code is currently functional, but possibly not optimized
  
  Steps to transfer code to a new arm/manipulator:
  1) Update all relevant constants in Manipulator.hpp
  2) See how angle information is gathered (potentiometers, encoders, etc.)
  3) Make sure that degrees increase how they're supposed to (should not decreasing if they should increase, etc.) and modify getDegrees() as necessary
  4) Check ports
  5) See how limit information is gathered and modify isLimit() as necessary
  6) Test stuff
  7) Add wrist/manipulator code (Main and Secondary Axis are already in the code)
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
double xPos = 9.25;
double yPos = 0;

void Robot::RobotInit() 
{
}

void Robot::RobotPeriodic() 
{
  
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {
  Manip.initializePID(true); //PID is initialized
}
//Highly recommended to reset to encoder both axes before starting
void Robot::TeleopPeriodic() {
  double yAxisMain = Blitz_Joy.getAxis(1,0); 
  double yAxisSecondary = Blitz_Joy.getAxis(1,1);
  xPos = (Blitz_Joy.getAxis(3, 1) * 3) + 12.5;
  yPos = (Blitz_Joy.getAxis(3, 0) * 10) + 10;

  if (Blitz_Joy.getButton(3, 0))
  {
    if (Manip.isPossible(xPos, yPos))
    {
      Manip.moveToCoordinates(xPos, yPos);
    }
    else
    {
      Manip.manipSet(0,0);
      Manip.manipSet(0,1);
    }
  }
  else
  {
    //Main Axis
    if (Blitz_Joy.getButton(1,0)) //Reset To Encoder
    {
      Manip.resetToEncoder(0);
    }
    else if (Blitz_Joy.getButton(2,0))
    {
      Manip.manipSetToDegrees(180, 0);
    }
    else
    {
      if (yAxisMain > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisMain < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
      {
        Manip.manipSet(yAxisMain,0);
      }
      else
      {
        Manip.manipSet(0,0);
      }
    }

    //Secondary Axis
    if (Blitz_Joy.getButton(1,1))
    {
      Manip.resetToEncoder(1);
    }
    else if (Blitz_Joy.getButton(2,1))
    {
      Manip.manipSetToDegrees(180, 1);
    }
    else
    {
      if (yAxisSecondary > Blitz_Joy.JOYSTICK_DEAD_ZONE || yAxisSecondary < -Blitz_Joy.JOYSTICK_DEAD_ZONE)
      {
        Manip.manipSet(yAxisSecondary,1);
      }
      else
      {
        Manip.manipSet(0,1);
      }
    }
  }
  
  frc::SmartDashboard::PutNumber("Degrees Main", Manip.getDegrees(0));
  frc::SmartDashboard::PutNumber("Raw Encoder Main", Manip.Main_Axis.GetSelectedSensorPosition(0));
  frc::SmartDashboard::PutNumber("Degrees Secondary", Manip.getDegrees(1));
  frc::SmartDashboard::PutNumber("Raw Encoder Secondary", Manip.Secondary_Axis.GetSelectedSensorPosition(0));
  frc::SmartDashboard::PutNumber("X-Pos", xPos);
  frc::SmartDashboard::PutNumber("Y-Pos", yPos);

/*
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
  */
  frc::Wait(0.005);
}
void Robot::TestPeriodic() {
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif