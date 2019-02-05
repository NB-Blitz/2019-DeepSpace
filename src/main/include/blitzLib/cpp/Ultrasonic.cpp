#include "Ultrasonic.hpp"

using namespace std;

double Blitz::Ultrasonic::getDistance()
{   
    return ((ultrasonic.GetVoltage() / (double)(5.0 / 512.0)) / 12.0);
}

bool Blitz::Ultrasonic::willCrash()
{
    return getDistance() <= MIN_DISTANCE;
}