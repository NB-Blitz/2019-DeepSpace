#pragma once

#include <frc/WPILib.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

using namespace std;
using namespace frc;
using namespace nt;

namespace Blitz
{
    class LineTrack
    {
        public:
            LineTrack()
            {
                auto instance = NetworkTableInstance::GetDefault();
                auto table = instance.GetTable(TABLE_NAME);
                alignX = table->GetEntry("alignX");
                alignZ = table->GetEntry("alignZ");
            }
            double * GetDirections(); // {joystickX, joystickY, joystickZ}
            void Update();
        private:
            const string TABLE_NAME = "BlitzTableOfDoomTM";

            const double SPEED_H = 0.5;
            const double SPEED_R = 0.2;
            const double SPEED = 1;

            const double DEAD_ZONE = 0.1;
            const double DEAD_ZONE_R = 3;

            double directions[3];
            NetworkTableEntry alignX;
            NetworkTableEntry alignZ;
    };
}