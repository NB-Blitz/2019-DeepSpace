#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>

namespace Blitz
{
    class Climber
    {
        public:
            Climber();

            void StartCompressor();
            void SetFrontSolenoid(bool Up);
            void SetBackSolenoid(bool Up);

        private:
            frc::Compressor comp;
            frc::Solenoid FrontSolenoid, BackSolenoid;
    };
}