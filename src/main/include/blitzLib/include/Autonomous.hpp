#pragma once

#include <frc/WPILib.h>
#include "AHRS.h"

#include "DataTypes.hpp"
#include "DriveReferences.hpp"

using namespace frc;

namespace Blitz {

    class Autonomous {

        public:
            Autonomous()
            {
                seconds.Start();
            }

            Blitz::Models::MecanumInput Run(double Yaw);

        private:
            double Speed = 0.3;

            bool direction = true; //True starts on the left, false on the right

            double stageOne = 1;
            double stageTwo = 2.6;
            double stageThree = 2.6;
            double stageFour = 2.7;
            double stageFive = 5;
            double stageSix = 5.1;
            double stageSeven = 6.5;

            double turnAngle = 180;
            frc::Timer seconds; 
            Blitz::Models::MecanumInput MecanumInput;
    
    };
}