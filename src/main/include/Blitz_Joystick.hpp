#ifndef SRC_BLITZ_JOYSTICK_HPP_
#define SRC_BLITZ_JOYSTICK_HPP_

//Won't be necessary once xBox is implemented
enum joystick
{
  Shoulder_Joystick = 0,
  Elbow_Joystick = 1,
  Wrist_Joystick = 2
};

enum joystickAxes
{
  X_Axis = 0,
  Y_Axis = 1,
  Z_Axis = 2
};

enum xBoxButton
{
    XBox_A_Button = 0,
    XBox_B_Button = 1,
    XBox_X_Button = 2,
    XBox_Y_Button = 3,
    XBox_Left_Bumper = 4,
    XBox_Right_Bumper = 5,
    XBox_Back_Button = 6, //Small button in center left
    XBox_Start_Button = 7 //Small button in center right
};

enum xBoxAxes
{
    XBox_Left_X_Axis = 0,
    XBox_Left_Y_Axis = 1,
    XBox_Left_Trigger = 2,
    XBox_Right_Trigger = 3,
    XBox_Right_X_Axis = 4,
    XBox_Right_Y_Axis = 5
};

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc
{
    class Blitz_Joystick
    {
        private:
            Joystick Joystick_Main, Joystick_Secondary;
            XboxController XBox;

        public:
            Blitz_Joystick();
            bool getButton(int id, int axisID);
            bool getXBoxButton(int id);
            double getAxis(int id, int axisID);
            double getXBoxAxis(int id);
            double const JOYSTICK_DEAD_ZONE = 0.1;
            
    };
}

#endif