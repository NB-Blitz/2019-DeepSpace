/*
  2/23/19
  Instructions for Use:
  1) Update maxRange and minRange pseudo-constants (in Robot.h) to what the arm is truly capable of
  2) Move each joint with a potentiometer to its lowest and highest possible points (as seen in step 1)
  3) Update code so that it accounts for inverted motors/potentiometers
  4) Press the "A" button on the Xbox controller, then record the values showm on the SmartDashboard
*/

#include "Robot.h"

Robot::Robot() :
  Xbox(0),
  AutoManager(),
  Manip()
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
  //Get Home Values
  homeEncoderValueShoulder = Manip.getRawUnits(Shoulder_Axis);
  homeEncoderValueElbow = Manip.getRawUnits(Elbow_Axis);
  homeEncoderValueWrist = Manip.getRawUnits(Wrist_Axis);   
  
  //Display Home Values on SmartDashboard
  frc::SmartDashboard::PutNumber("Home Encoder - Shoulder", homeEncoderValueShoulder);
  frc::SmartDashboard::PutNumber("Home Encoder - Elbow", homeEncoderValueElbow);
  frc::SmartDashboard::PutNumber("Home Encoder - Wrist", homeEncoderValueWrist);

  while (IsOperatorControl() && IsEnabled()) 
  {
    //Receive Xbox input
    Xbox.update();
    axisShoulder = Xbox.LeftY;
    axisElbow = Xbox.RightY;
    axisWrist = Xbox.LeftX;
    
    //Move joints
    //Manip.manipSet(0.4*axisShoulder, Shoulder_Axis, homeEncoderValueShoulder); //Gearbox is having issues
    Manip.manipSet(0.4*axisElbow, Elbow_Axis, homeEncoderValueElbow);
    Manip.manipSet(axisWrist, Wrist_Axis, homeEncoderValueWrist);
    
    //Receive raw input from potentiometers
    rawShoulder = Manip.getRawUnits(Shoulder_Axis);
    rawElbow = Manip.getRawUnits(Elbow_Axis);
    rawWrist = Manip.getRawUnits(Wrist_Axis);

    //Record remaining information to SmartDashboard
    frc::SmartDashboard::PutNumber("Shoulder Pot Raw Current", rawShoulder);
    frc::SmartDashboard::PutNumber("Shoulder Pot Degrees", Manip.getDegrees(Shoulder_Axis, homeEncoderValueShoulder));
    frc::SmartDashboard::PutNumber("Elbow Pot Raw Current", rawElbow);
    frc::SmartDashboard::PutNumber("Elbow Pot Degrees", Manip.getDegrees(Elbow_Axis, homeEncoderValueElbow));
    frc::SmartDashboard::PutNumber("Wrist Pot Raw Current", rawWrist);
    frc::SmartDashboard::PutNumber("Wrist Pot Degrees", Manip.getDegrees(Wrist_Axis, homeEncoderValueWrist));
    frc::SmartDashboard::PutNumber("Shoulder's Axis", axisShoulder);
    frc::SmartDashboard::PutNumber("Elbow's Axis", axisElbow);
    frc::SmartDashboard::PutNumber("Wrist's Axis", axisWrist);
    
    //Delay
    frc::Wait(0.005);
  }
}
void Robot::Test() //This is test code using the xBox Controller (for some reason, it won't run on Test mode, but the toggles work correctly when put in OperatorControl())
{

}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
