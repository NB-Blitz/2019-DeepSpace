#include "Ultrasonic.hpp"

using namespace std;

double Blitz::Ultrasonic::getDistance()
{   
    if ((ultrasonicA.GetVoltage() / MULTIPLIER) / 12.0 < (ultrasonicB.GetVoltage() / MULTIPLIER) / 12.0)
        return ((ultrasonicA.GetVoltage() / MULTIPLIER) / 12.0);
    else
        return ((ultrasonicB.GetVoltage() / MULTIPLIER) / 12.0);
}

bool Blitz::Ultrasonic::willCrash()
{
    SmartDashboard::PutNumber("Distance", getDistance());
    return getDistance() <= (MIN_DISTANCE / 12.0);
}