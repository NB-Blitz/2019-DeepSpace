#ifndef SRC_BLITZ_JOYSTICK_HPP_
#define SRC_BLITZ_JOYSTICK_HPP_

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