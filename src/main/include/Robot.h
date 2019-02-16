#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>
#include <Manipulator.hpp>
#include <Blitz_Joystick.hpp>


class Robot : public frc::SampleRobot 
{
    public:
        Robot();
        void RobotInit() override;
        void Autonomous() override;
        void OperatorControl() override;
        void Test() override;

    private:

        //Test Variable -> true for limit switches, false for pre-set known
        bool areLimits = false;

        //if the previous variable is true, these variables are relevant
        bool initialReset = false;

        //if the previous variable is false, these variables are relevant
        double homeEncoderValueShoulder, homeEncoderValueElbow;
        double yAxisShoulder, yAxisElbow;
        frc::Manipulator Manip;
        frc::Blitz_Joystick Blitz_Joy;
};
