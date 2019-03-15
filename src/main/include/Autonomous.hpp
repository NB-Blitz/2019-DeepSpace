#pragma once

#include <frc/WPIlib.h> 
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