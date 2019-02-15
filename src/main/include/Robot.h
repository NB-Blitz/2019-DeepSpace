#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>
#include "AHRS.h"


class Robot : public frc::SampleRobot 
{
    public:
        Robot();

        void RobotInit() override;
        void Autonomous() override;
        void OperatorControl() override;
        void Test() override;
        void FieldControl(double x, double y);

    private:
        TalonSRX LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor;
        AHRS Navx;

        Blitz::BlitzLogger Logger;
        Blitz::Models::DriveMotors Motors;
        Blitz::Models::MecanumInput MecanumInput;
        Blitz::Mecanum MecanumDrive;
        Blitz::Joysticks::XboxController Xbox;

       //for field oriented control 
        double newX = 0;
        double newY = 0;
};
