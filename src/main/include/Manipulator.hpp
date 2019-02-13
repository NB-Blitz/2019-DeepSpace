#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>

namespace Blitz
{
    class Manipulator
    {
        public:

            Manipulator();

            void ResetPosition();
            void MoveManipulatorSpeed(double speed);
            void MoveManipulatorPosition(double diameter);

        private:
            const int SMALL_GEAR_TEETH = 24;
            const int LARGE_GEAR_TEETH = 57;
            const int PULSES_PER_ROTATION = 90;

            const double PULSES_PER_ANGLE_SMALL_GEAR = 360/((SMALL_GEAR_TEETH/LARGE_GEAR_TEETH) * PULSES_PER_ROTATION);

            double currentPosition = 0;
            int direction = 1;

            frc::DigitalInput LimitSwitch;
            frc::Counter PositionCounter;
            TalonSRX ClawTalon;
    };
}