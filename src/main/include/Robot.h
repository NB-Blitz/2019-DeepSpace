#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>


class Robot : public frc::SampleRobot 
{
    public:
        Robot();

        void RobotInit() override;
        void Autonomous() override;
        void OperatorControl() override;
        void Test() override;

    private:
        TalonSRX LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor;

        Blitz::BlitzLogger Logger;
        Blitz::Models::DriveMotors Motors;
        Blitz::Models::MecanumInput MecanumInput;
        Blitz::Mecanum MecanumDrive;
        Blitz::Joysticks::XboxController Xbox;
        Blitz::Ultrasonic Ultrasonic;
};
