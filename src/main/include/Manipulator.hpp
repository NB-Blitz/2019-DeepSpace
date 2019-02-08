#pragma once

#include <frc/WPILib.h>

namespace Blitz
{
    class Manipulator
    {
        public:
            Manipulator();
            
            frc::Solenoid ArmsIn, ArmsOut;

            void CloseArms();
            void OpenArms();
    };
}