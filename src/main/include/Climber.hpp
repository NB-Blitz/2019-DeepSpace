#pragma once

#include <frc/WPILib.h>
#include <blitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>

namespace Blitz
{
    class Climber
    {
        public:
            Climber();

            void StartCompressor();
            void SetBackSolenoid(bool Up);

        private:
            frc::Compressor comp;
            frc::Solenoid BackSolenoid;
    };
}