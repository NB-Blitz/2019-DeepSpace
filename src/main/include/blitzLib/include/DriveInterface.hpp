#pragma once

#include <frc/WPILib.h>

#include "DataTypes.hpp"
#include "DriveReferences.hpp"

using namespace frc;

namespace Blitz
{
    class DriveInterface 
    {
        public:
            virtual void Initialize() = 0;
            virtual void Drive() = 0;
            virtual void Close() = 0;
            virtual void SetMotorDirection() = 0;
    };
}