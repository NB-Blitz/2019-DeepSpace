#include "JoySticks.hpp"

void Blitz::Joysticks::ThreeAxisJoystick::update()
{
    //Update Buttons
    Trigger = ThreeAxis.GetRawButton(TRIGGER_ID);
    Button2 = ThreeAxis.GetRawButton(BUTTON2_ID);
    Button3 = ThreeAxis.GetRawButton(BUTTON3_ID);
    Button4 = ThreeAxis.GetRawButton(BUTTON4_ID);
    Button5 = ThreeAxis.GetRawButton(BUTTON5_ID);
    Button6 = ThreeAxis.GetRawButton(BUTTON6_ID);
    Button7 = ThreeAxis.GetRawButton(BUTTON7_ID);
    Button8 = ThreeAxis.GetRawButton(BUTTON8_ID);
    Button9 = ThreeAxis.GetRawButton(BUTTON9_ID);
    Button10 = ThreeAxis.GetRawButton(BUTTON10_ID);
    Button11 = ThreeAxis.GetRawButton(BUTTON11_ID);
    Button12 = ThreeAxis.GetRawButton(BUTTON12_ID);
    
    //Update DPad
    DPad[0] = ThreeAxis.GetRawButton(D_PAD_ID[0]);
    DPad[1] = ThreeAxis.GetRawButton(D_PAD_ID[1]);
    DPad[2] = ThreeAxis.GetRawButton(D_PAD_ID[2]);
    DPad[3] = ThreeAxis.GetRawButton(D_PAD_ID[3]);
    DPad[4] = ThreeAxis.GetRawButton(D_PAD_ID[4]);
    DPad[5] = ThreeAxis.GetRawButton(D_PAD_ID[5]);
    DPad[6] = ThreeAxis.GetRawButton(D_PAD_ID[6]);
    DPad[7] = ThreeAxis.GetRawButton(D_PAD_ID[7]);

    //Update Axis
    XAxis = ThreeAxis.GetRawAxis(X_AXIS_ID);
    YAxis = ThreeAxis.GetRawAxis(Y_AXIS_ID);
    Dial = ThreeAxis.GetRawAxis(DIAL_ID);
}