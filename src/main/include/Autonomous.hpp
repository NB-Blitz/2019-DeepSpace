#pragma once

#include <frc/WPILib.h> 
#include <blitzLib/BlitzLib.hpp>

namespace Blitz
{
    class Autonomous
    {
        public:
            Autonomous();

            bool DriveToBall(Blitz::Models::MecanumInput *Input);

        private:

    };
}