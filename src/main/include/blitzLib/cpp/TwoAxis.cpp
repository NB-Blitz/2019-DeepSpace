#include "JoySticks.hpp"

void Blitz::Joysticks::TwoAxisJoystick::update()
{
    //Update Buttons
    Trigger = TwoAxis.GetRawButton(TRIGGER_ID);
    Button2 = TwoAxis.GetRawButton(BUTTON2_ID);
    Button3 = TwoAxis.GetRawButton(BUTTON3_ID);
    Button4 = TwoAxis.GetRawButton(BUTTON4_ID);
    Button5 = TwoAxis.GetRawButton(BUTTON5_ID);
    Button6 = TwoAxis.GetRawButton(BUTTON6_ID);
    Button7 = TwoAxis.GetRawButton(BUTTON7_ID);
    Button8 = TwoAxis.GetRawButton(BUTTON8_ID);

    //Update Axis
    XAxis = TwoAxis.GetRawAxis(X_AXIS_ID);
    YAxis = TwoAxis.GetRawAxis(Y_AXIS_ID);
    Dial = TwoAxis.GetRawAxis(DIAL_ID);
}