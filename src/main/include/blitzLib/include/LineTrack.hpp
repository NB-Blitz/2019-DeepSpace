#pragma once

#include <frc/WPILib.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

using namespace std;
using namespace frc;

namespace Blitz
{
    class LineTrack
    {
        public:
            LineTrack()
            {
                
            }
        private:
            const string TABLE_NAME = "BlitzTableOfDoomTM";
            
    };
}