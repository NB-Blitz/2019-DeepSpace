/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
 
}

void Robot::RobotInit() {
  
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::Autonomous()
{
 
}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl()
 {

    frc::SmartDashboard::PutNumber("FrontLeftEncoder", -LeftFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackLeftEncoder", -LeftBackMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("FrontRightEncoder", RightFrontMotor.GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("BackRightEncoder", RightBackMotor.GetSelectedSensorVelocity(0));

    frc::Wait(0.005);
  }
}

/**
 * Runs during test mode
 */
void Robot::Test() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
