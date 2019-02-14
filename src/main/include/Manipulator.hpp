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

            double currentPosition = 0;

        private:
            const double SMALL_GEAR_TEETH = 24;
            const double LARGE_GEAR_TEETH = 57;
            const double PULSES_PER_ROTATION = 180;

            const double PULSES_PER_ANGLE_SMALL_GEAR = ((PULSES_PER_ROTATION/LARGE_GEAR_TEETH) * SMALL_GEAR_TEETH)/360;

            int direction = 1;

            frc::DigitalInput LimitSwitch;
            frc::Counter PositionCounter;
            TalonSRX ClawTalon;
    };
}