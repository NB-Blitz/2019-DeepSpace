#include "JoySticks.hpp"

void Blitz::Joysticks::XboxController::update()
{
    //Update Buttons
    AButton = Xbox.GetRawButton(A_BUTTON_ID);
    BButton = Xbox.GetRawButton(B_BUTTON_ID);
    XButton = Xbox.GetRawButton(X_BUTTON_ID);
    YButton = Xbox.GetRawButton(Y_BUTTON_ID);
    LeftBumper = Xbox.GetRawButton(LEFT_BUMPER_ID);
    RightBumper = Xbox.GetRawButton(RIGHT_BUMPER_ID);
    LeftStickButton = Xbox.GetRawButton(LEFT_STICK_BUTTON_ID);
    RightStickButton = Xbox.GetRawButton(RIGHT_STICK_BUTTON_ID);

    //Update Axis
    LeftX = Xbox.GetRawAxis(LEFT_X_AXIS_ID);
    LeftY = Xbox.GetRawAxis(LEFT_Y_AXIS_ID);
    RightX = Xbox.GetRawAxis(RIGHT_X_AXIS_ID);
    RightY = Xbox.GetRawAxis(RIGHT_Y_AXIS_ID);
    LeftTrigger = Xbox.GetRawAxis(LEFT_TRIGGER_ID);
    RightTrigger = Xbox.GetRawAxis(RIGHT_TRIGGER_ID);
    
}