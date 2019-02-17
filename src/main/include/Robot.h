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
        //if the previous variable is false, these variables are relevant
        double homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist;
        double yAxisShoulder, yAxisElbow;
        frc::Manipulator Manip;
        frc::Blitz_Joystick Blitz_Joy;
};
