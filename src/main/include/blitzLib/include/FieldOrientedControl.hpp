#pragma once

#include <frc/WPILib.h>
#include "AHRS.h"
#include "Mecanum.hpp"

using namespace std;
using namespace frc;

namespace Blitz 
{
    class FieldOrientedControl 
    {
        public:
            FieldOrientedControl()
            {

            }

            Blitz::Models::MecanumInput FieldControl(double x, double y, double Yaw);
    };
}