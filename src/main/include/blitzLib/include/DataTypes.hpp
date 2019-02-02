#pragma once

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

namespace Blitz
{
    namespace Models 
    { 
        //The 4 Motors For the Drive Train
        class DriveMotors
        {
            public:
                TalonSRX * Motor1; // Front Left
                TalonSRX * Motor2; // Front Right
                TalonSRX * Motor3; // Back Left
                TalonSRX * Motor4; // Back Right

                DriveMotors() {}

                DriveMotors(TalonSRX * LeftFrontMotor, TalonSRX * LeftBackMotor, TalonSRX * RightFrontMotor, TalonSRX * RightBackMotor)
                {
                    this->Motor1 = LeftFrontMotor;
                    this->Motor2 = LeftBackMotor;
                    this->Motor3 = RightFrontMotor;
                    this->Motor4 = RightBackMotor;
                }            
        };

        // Joystick Data for Arcade Drive
        class ArcadeInput
        {
            public:
                double YValue; // Speed
                double ZValue; // Rotation

                ArcadeInput(double YValue, double XValue)
                {
                    this->YValue = YValue;
                    this->ZValue = ZValue;
                }

                ArcadeInput()
                {
                    YValue = 0;
                    ZValue = 0;
                }
        };

        // Joystick Data for Tank Drive
        class TankInput
        {
            public:
                double LeftValue;  // Left Joystick Speed
                double RightValue; // Right Joystick Speed

                TankInput(double LeftValue, double RightValue)
                {
                    this->LeftValue = LeftValue;
                    this->RightValue = RightValue;                        
                }                                

                TankInput()
                {
                    LeftValue = 0;
                    RightValue = 0;
                }
        };

        // Joystick Data for Mecanum Drive
        class MecanumInput
        {
            public:
                double XValue; // Left/Right Speed
                double YValue; // Forward/Back Speed
                double ZValue; // Rotation Speed 

                MecanumInput(double XValue, double YValue, double ZValue)
                {
                    this->XValue = XValue;
                    this->YValue = YValue;
                    this->ZValue = ZValue;
                }

                MecanumInput()
                {
                    XValue = 0;
                    YValue = 0;
                    ZValue = 0;
                }
        };

        // Joystick Data for Ocotocanum Drive
        class OctocanumInput
        {
            public:
                double XValue; // Left/Right Speed (ignored in tank mode)
                double YValue; // Forward/Back Speed
                double ZValue; // Rotation Speed 
                int DriveMode; //Drive Mode

                OctocanumInput(double XValue, double YValue, double ZValue, int DriveMode)
                {
                    this->XValue = XValue;
                    this->YValue = YValue;
                    this->ZValue = ZValue;
                    this-> DriveMode = DriveMode;
                }

                OctocanumInput()
                {
                    XValue = 0;
                    YValue = 0;
                    ZValue = 0;
                    DriveMode = 0;
                }
        };
    }
}