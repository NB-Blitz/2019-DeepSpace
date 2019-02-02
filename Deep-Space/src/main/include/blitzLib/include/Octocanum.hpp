#pragma once

#include <frc/WPILib.h>

#include "DataTypes.hpp"
#include "BlitzLogger.hpp"
#include "DriveReferences.hpp"

using namespace std;
using namespace frc;

namespace Blitz
{
    class Octocanum
    {
        public:
            Octocanum(Blitz::Models::DriveMotors *Motors,  Blitz::BlitzLogger *Logger)
            {
                this->Logger = Logger;
                this->Motors = Motors;
            }

            void SetMotorDirection(int Motor, int dir);
            void TuneF(int MotorID, double FGain);
            void TuneP(int MotorID, double PGain);
            void TuneI(int MotorID, double IGain);
            void TuneD(int MotorID, double DGain);

            double GetMotorOutput(int MotorID);

            void Initialize(Blitz::Models::OctocanumInput *Input);
            void Run();
            void Close();
            
            bool UsePID = false;
            
        private:
            Blitz::BlitzLogger *Logger; 
            Blitz::Models::OctocanumInput *InputData;
            Blitz::Models::DriveMotors *Motors;
            int MotorDirs[4] = {1, 1, 1, 1};
            double motorValues[4] = {0, 0, 0, 0};

    };
}