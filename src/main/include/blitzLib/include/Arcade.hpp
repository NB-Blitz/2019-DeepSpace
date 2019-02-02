#pragma once

#include <frc/WPILib.h>

#include "DataTypes.hpp"
#include "BlitzLogger.hpp"
#include "DriveReferences.hpp"

using namespace std;
using namespace frc;

namespace Blitz
{
    class Arcade
    {
        public:
            Arcade(Blitz::Models::DriveMotors *Motors, Blitz::BlitzLogger *Logger)
            {
                this->Motors = Motors;
                this->Logger = Logger;
            }

            void SetMotorDirection(int Motor, int dir);
            void Initialize(Blitz::Models::ArcadeInput *Input);
            void Run();
            void Close();
            
            bool UsePID = false;

        private:
            Blitz::BlitzLogger *Logger;
            Blitz::Models::ArcadeInput *InputData;
            Blitz::Models::DriveMotors *Motors;
            int MotorDirs[4] = {1, 1, 1, 1};
    };
}