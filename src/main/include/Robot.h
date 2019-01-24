/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//C++ Library Include
#include <string>

//WPILIB Include
#include <WPILib.h>

//Blitz Lib Include
#include "blitzlib/BlitzLib.hpp"

/**

 */
class Robot : public frc::SampleRobot 
{
 public:
  Robot();

  void RobotInit() override;
  void Autonomous() override;
  void OperatorControl() override;
  void Test() override;

 private:

};
