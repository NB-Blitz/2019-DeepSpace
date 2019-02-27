#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>
#include <Manipulator.hpp>
#include <Blitz_Joystick.hpp>
#include <Math.h>
#include "Autonomous.hpp"

class Robot : public frc::SampleRobot 
{
    public:
        Robot();

        void RobotInit() override;
        void Autonomous() override;
        void OperatorControl() override;
        void Test() override;

    private:
        Blitz::Joysticks::XboxController Xbox;
        Blitz::Autonomous AutoManager;
        Blitz::Manipulator Manip;
        double homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist;
        double axisShoulder, axisElbow, axisWrist;
        double rawShoulder = 0, rawElbow = 0, rawWrist = 0;
        const double SPEED_MULTIPLIER_SHOULDER = 0.4; //Maximum speed for shoulder
        const double SPEED_MULTIPLIER_ELBOW = 0.2; //Maximum speed for elbow
        const double SPEED_MULTIPLIER_WRIST = 1; //Maximum speed for wrist
        
};

