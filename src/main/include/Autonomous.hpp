#pragma once

#include <frc/WPIlib.h> 
#include <BlitzLib/BlitzLib.hpp>

namespace Blitz
{
    class Autonomous
    {
        public:
            Autonomous();

            void DriveToBall(Blitz::Models::MecanumInput *Input);

        private:

    };
}