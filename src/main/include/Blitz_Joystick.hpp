#ifndef SRC_BLITZ_JOYSTICK_HPP_
#define SRC_BLITZ_JOYSTICK_HPP_

#include "frc/WPILib.h"


namespace frc
{
	class Blitz_Joystick
	{
        private:
            frc::Joystick joystick;
            const double JOYSTICK_DEAD_ZONE = 0.2; //On either side of center, so real range of inactivity is twice that of the dead zone
	    public:
            Blitz_Joystick();
            int getJoystick(); //Returns how the joystick values will change (-1 is decreasing, +1 is increasing, 0 is no change)
            bool getButton(int id);

	};
}

#endif 