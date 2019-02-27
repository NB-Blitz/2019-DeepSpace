/*
  2/26/19
  - Coefficients are good as far as I can tell
  - Extraneous code temporarily removed (old code still saved in my system, so methods for the rest of the robot will be added later)
  - Currently testing moving to an angular position, which will quickly be followed by movement to coordinates of interest

  NOTE: Start in home position to guarantee accuracy with degrees
  Shoulder: 320 (about 37 minutes on a clock)
  Elbow: 40 
  Wrist: 90
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
    
    if (Xbox.AButton)
    {
      //Move all joints so that the arm sticks out horizontally
      Manip.manipSetToDegrees(270, Shoulder_Axis, homeEncoderValueShoulder);
      Manip.manipSetToDegrees(180, Elbow_Axis, homeEncoderValueElbow);
      Manip.manipSetToDegrees(180, Wrist_Axis, homeEncoderValueWrist);
    }
    else
    {
      //Move joints manually
      Manip.manipSet(SPEED_MULTIPLIER_SHOULDER * axisShoulder, Shoulder_Axis, homeEncoderValueShoulder); //Gearbox is having issues
      Manip.manipSet(SPEED_MULTIPLIER_ELBOW * axisElbow, Elbow_Axis, homeEncoderValueElbow);
      Manip.manipSet(SPEED_MULTIPLIER_WRIST * axisWrist, Wrist_Axis, homeEncoderValueWrist);
    }
    
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
    frc::SmartDashboard::PutBoolean("IsHorizontal", Xbox.AButton);
    
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
