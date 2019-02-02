#pragma once

#include <frc/WPILib.h>

using namespace frc;

namespace Blitz
{
    class DriveReference
    {
        public:
            static constexpr double WHEEL_RADIUS = 0.0889; //Radius of the wheel in Meters
            static constexpr double ENCODER_UNITS_PER_ROTATION = 600;
            static constexpr double MAX_RPM = 300;

            static constexpr double WHEEL_CIRCUMFERENCE = 3.14 * (2 * WHEEL_RADIUS);
            static constexpr double ROTATIONS_PER_METER = 1/WHEEL_CIRCUMFERENCE;
            static constexpr double ENCODER_UNITS_PER_METER = ROTATIONS_PER_METER * ENCODER_UNITS_PER_ROTATION;
            static constexpr double SECOND_TO_HUNDERD_MILLISECOND_CONVERSION = 10;

            static constexpr double MAX_SPEED_METERS_PER_SECOND = (MAX_RPM/60) * ROTATIONS_PER_METER;
            static constexpr double MAX_SPEED_COUNTS_PER_HUNDRED_MILLISECONDS = ((MAX_RPM/60)/SECOND_TO_HUNDERD_MILLISECOND_CONVERSION)* ENCODER_UNITS_PER_ROTATION;


            static constexpr double MOTOR1_kF = 3;
            static constexpr double MOTOR1_kP = .25;
            static constexpr double MOTOR1_kI = 0;
            static constexpr double MOTOR1_kD = 0;

            static constexpr double MOTOR2_kF = 3;
            static constexpr double MOTOR2_kP = .25;
            static constexpr double MOTOR2_kI = 0;
            static constexpr double MOTOR2_kD = 0;

            static constexpr double MOTOR3_kF = 3;
            static constexpr double MOTOR3_kP = .25;
            static constexpr double MOTOR3_kI = 0;
            static constexpr double MOTOR3_kD = 0;

            static constexpr double MOTOR4_kF = 3;
            static constexpr double MOTOR4_kP = .25;
            static constexpr double MOTOR4_kI = 0;
            static constexpr double MOTOR4_kD = 0;

        private:

    };
}