#include <Autonomous.hpp>

Blitz::Autonomous::Autonomous()
{

}

bool Blitz::Autonomous::DriveToBall(Blitz::Models::MecanumInput *Input)
{
    double XCenterDist = SmartDashboard::GetNumber("XOffset", 0.0);
    double YCenterDist = SmartDashboard::GetNumber("YOffset", 0.0);
    double BallDist = SmartDashboard::GetNumber("Distance", 0.0);

    double XInput = 0;
    double YInput = -(BallDist - 35) / 120;
    double ZInput = pow(XCenterDist / 176, 3)/2.5;

    if(BallDist == 0)
    {
        YInput = 0;
    }

    Input->XValue = (XInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    Input->YValue = (YInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);
    Input->ZValue = (ZInput * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND);

    return BallDist < 35 && BallDist != 0;
}