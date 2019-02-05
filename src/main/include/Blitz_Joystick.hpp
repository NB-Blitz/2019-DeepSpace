#ifndef SRC_BLITZ_JOYSTICK_HPP_
#define SRC_BLITZ_JOYSTICK_HPP_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc
{
    class Blitz_Joystick
    {
        private:
            Joystick joystick;
            const double JOYSTICK_DEAD_ZONE = 0.1;

        public:
            Blitz_Joystick();
            bool getButton(int id);
            double getAxis(int id);
    };
}

#endif