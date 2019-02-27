#pragma once

#include <frc/WPILib.h>
#include <BlitzLib/BlitzLib.hpp>
#include <ctre/Phoenix.h>
#include <Manipulator.hpp>
#include <Math.h>
#include "AHRS.h"
#include "Autonomous.hpp"
#include "Manipulator.hpp"

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
        AHRS Navx;

        Blitz::BlitzLogger Logger;
        Blitz::Models::DriveMotors Motors;
        Blitz::Models::MecanumInput MecanumInput;
        Blitz::Mecanum MecanumDrive;
        Blitz::Joysticks::XboxController Xbox;
        Blitz::FieldOrientedControl FieldControl;
        Blitz::Autonomous AutoManager;
        Blitz::Ultrasonic Ultrasonics;
        Blitz::LineTrack LineTracker;
        Blitz::Manipulator Manipulator;
        
        double homeEncoderValueShoulder, homeEncoderValueElbow, homeEncoderValueWrist;
        double axisShoulder, axisElbow, axisWrist;
        double rawShoulder = 0, rawElbow = 0, rawWrist = 0;
        const double SPEED_MULTIPLIER_SHOULDER = 0.4; //Maximum speed for shoulder
        const double SPEED_MULTIPLIER_ELBOW = 0.2; //Maximum speed for elbow
        const double SPEED_MULTIPLIER_WRIST = 1; //Maximum speed for wrist
        
};

