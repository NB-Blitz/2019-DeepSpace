#ifndef SRC_MANIPULATOR_HPP_
#define SRC_MANIPULATOR_HPP_

#include "frc/WPILib.h"


namespace frc
{
	class Manipulator
	{
        private:
        /*
            frc::PWMTalonSRX Chuck, Wrist;
            frc::Encoder Chuck_Encoder, Wrist_Encoder;
        */
            frc::PWMTalonSRX Main_Axis;
            frc::DigitalInput Main_Axis_Limit_Switch;
            const double TO_DEGREES = 10; //Converts from Encoder Units to Degrees (NOT Tested)
            const double DEAD_ZONE = 5; //On either side of center, so real range of motion is twice that of the dead zone
	    public:
            Manipulator();
            double getDegrees();
            void reset(); //Moves to 0 degrees
            void moveTo(double degrees);

	};
}

#endif 