#pragma once

#include <frc/WPIlib.h> 
#include <BlitzLib/BlitzLib.hpp>

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