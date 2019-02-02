#pragma once

#include <frc/WPILib.h>

using namespace frc;

namespace Blitz
{
    namespace Joysticks
    {
        class XboxController
        {
            public:
                int port;

                Joystick Xbox;

                //Buttons
                double AButton = 0;
                double BButton = 0;
                double XButton = 0;
                double YButton = 0;
                double LeftBumper = 0;
                double RightBumper = 0;
                double LeftStickButton = 0;
                double RightStickButton = 0;

                //axis
                double LeftX = 0;
                double LeftY = 0;
                double RightX = 0;
                double RightY = 0;
                double LeftTrigger = 0;
                double RightTrigger = 0;

                //Button ids
                const int A_BUTTON_ID = 0;
                const int B_BUTTON_ID = 0;
                const int X_BUTTON_ID = 0;
                const int Y_BUTTON_ID = 0;
                const int LEFT_BUMPER_ID = 0;
                const int RIGHT_BUMPER_ID = 0;
                const int LEFT_STICK_BUTTON_ID = 0;
                const int RIGHT_STICK_BUTTON_ID = 0;

                //axis ids
                const int LEFT_X_AXIS_ID = 0;
                const int LEFT_Y_AXIS_ID = 0;
                const int RIGHT_X_AXIS_ID = 0;
                const int RIGHT_Y_AXIS_ID = 0;
                const int LEFT_TRIGGER_ID = 0;
                const int RIGHT_TRIGGER_ID = 0;

                void update();

                XboxController() :
                    Xbox(0)
                {

                }

                XboxController(int port):
                    Xbox(port)
                {
                    this->port = port;
                }


        };

        class ThreeAxisJoystick
        {
            public:
                int port = 0;

                Joystick ThreeAxis;

                //Buttons
                double Trigger = 0;
                double Button2 = 0;
                double Button3 = 0;
                double Button4 = 0;
                double Button5 = 0;
                double Button6 = 0;
                double Button7 = 0;
                double Button8 = 0;
                double Button9 = 0;
                double Button10 = 0;
                double Button11 = 0;
                double Button12= 0;
                double DPad[8] = {0, 0, 0, 0, 0, 0, 0, 0};

                //axis
                double XAxis = 0;
                double YAxis = 0;
                double zAxis = 0;
                double Dial = 0;

                //Button IDs
                const int TRIGGER_ID = 1;
                const int BUTTON2_ID = 2;
                const int BUTTON3_ID = 3;
                const int BUTTON4_ID = 4;
                const int BUTTON5_ID = 5;
                const int BUTTON6_ID = 6;
                const int BUTTON7_ID = 7;
                const int BUTTON8_ID = 8;
                const int BUTTON9_ID = 9;
                const int BUTTON10_ID = 10;
                const int BUTTON11_ID = 11;
                const int BUTTON12_ID = 12;
                const int D_PAD_ID[8] = {0, 0, 0, 0, 0, 0, 0, 0};

                //axis IDs
                const int X_AXIS_ID = 0;
                const int Y_AXIS_ID = 0;
                const int Z_AXIS_ID = 0;
                const int DIAL_ID = 0;

                void update();

                ThreeAxisJoystick() :
                    ThreeAxis(0)
                {

                }

                ThreeAxisJoystick(int port) :
                    ThreeAxis(port)
                {
                    this->port = port; 
                }
        };

        class TwoAxisJoystick
        {
            public:
                int port = 0;
                Joystick TwoAxis;

                //Buttons
                double Trigger = 0;
                double Button2 = 0;
                double Button3 = 0;
                double Button4 = 0;
                double Button5 = 0;
                double Button6 = 0;
                double Button7 = 0;
                double Button8 = 0;

                //Axis
                double XAxis = 0;
                double YAxis = 0;
                double Dial = 0;

                //Button IDs
                const int TRIGGER_ID = 1;
                const int BUTTON2_ID = 2;
                const int BUTTON3_ID = 3;
                const int BUTTON4_ID = 4;
                const int BUTTON5_ID = 5;
                const int BUTTON6_ID = 6;
                const int BUTTON7_ID = 7;
                const int BUTTON8_ID = 8;

                //Axis IDs
                const int X_AXIS_ID = 0;
                const int Y_AXIS_ID = 1;
                const int DIAL_ID = 2;

                void update();

                TwoAxisJoystick() :
                    TwoAxis(0)
                {

                }

                TwoAxisJoystick(int port) :
                    TwoAxis(port)
                {
                    this->port = port; 
                }

        };   
    }
}