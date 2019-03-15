#pragma once

#include <frc/WPILib.h>

using namespace frc;

namespace Blitz
{
    class Ultrasonic
    {
        public:
            Ultrasonic(int portA, int portB):
                ultrasonicA(portA),
                ultrasonicB(portB)
            {
                
            }
            
            double getDistance();
            bool willCrash();
        private:
            AnalogInput ultrasonicA;
            AnalogInput ultrasonicB;

            // Distance for robot to stop
            const int MIN_DISTANCE = 1; // Inches
            const double MULTIPLIER = (double)(5.0 / 512.0);
    };
}