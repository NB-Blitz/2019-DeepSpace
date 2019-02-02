#pragma once

#include <frc/WPILib.h>

#include "BlitzLogger.hpp"
#include "DataTypes.hpp"
#include "DriveReferences.hpp"

using namespace std;
using namespace frc;

namespace Blitz
{
    class Mecanum
    {

        public:
            Mecanum(Blitz::Models::DriveMotors *Motors,  Blitz::BlitzLogger *Logger)
            {
                this->Logger = Logger;
                this->Motors = Motors;
            }

            void SetMotorDirection(int Motor, int dir);
            void Initialize(Blitz::Models::MecanumInput *Input);
            void Drive();
            void Close();
            
            bool UsePID = false;
            
            
        private:
            Blitz::BlitzLogger *Logger; 
            Blitz::Models::MecanumInput *InputData;
            Blitz::Models::DriveMotors *Motors;
            int MotorDirs[4] = {1, 1, 1, 1};

    };
}