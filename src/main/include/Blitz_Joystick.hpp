#ifndef SRC_BLITZ_JOYSTICK_HPP_
#define SRC_BLITZ_JOYSTICK_HPP_

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

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc
{
    class Blitz_Joystick
    {
        private:
            Joystick Joystick_Main, Joystick_Secondary;

        public:
            Blitz_Joystick();
            bool getButton(int id, int axisID);
            double getAxis(int id, int axisID);
            double const JOYSTICK_DEAD_ZONE = 0.1;
    };
}

#endif