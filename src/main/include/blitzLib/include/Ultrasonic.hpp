#pragma once

#include <frc/WPILib.h>

using namespace frc;

namespace Blitz
{
    class Ultrasonic
    {
        public:
            Ultrasonic(int port):
                ultrasonic(port)
            {
                this->port = port;
            }
            
            double getDistance();
            bool willCrash();
        private:
            AnalogInput ultrasonic;
            int port;

            const int MIN_DISTANCE = 10;
    };
}