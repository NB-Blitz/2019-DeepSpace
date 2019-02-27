#pragma once

enum armAxis
{ 
  Shoulder_Axis = 0,
  Elbow_Axis = 1,
  Wrist_Axis = 2

};
enum customSpeed
{
  Off = 0,
  Max = 1
};
enum wristDegrees
{
    Parallel = 0,
    Straight_Down = 90,
    Straight_Up = -90
};

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <BlitzLib/BlitzLib.hpp>

namespace Blitz
{
    class Manipulator
    {
        public:

            Manipulator();

            void manipSet(double speed, int axisID, double rawHome); //PercentageOutput (No PID)
            void manipSetToDegrees(double degrees, int axisID, double rawHome);
            double getRawUnits(int axisID);
            double getDegrees(int axisID, double rawHome); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            void moveToCoordinates(double x, double y, double rawHomeShoulder, double rawHomeElbow); //In inches
            bool isPossible(double x, double y);
            void moveToXDegreesBelowParallel(double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist, double x);

        private:

            //For Shoulder Axis
            const double LENGTH_SHOULDER = 25;
            const double MIN_RANGE_SHOULDER = 200; //True Zero = faces ground
            const double MAX_RANGE_SHOULDER = 370;
            const double TO_DEGREES_SHOULDER = 0.4; //Placeholder
            const double HOME_POSITION_SHOULDER = 320;

            //For Elbow Axis
            const double LENGTH_ELBOW = 23;
            const double MIN_RANGE_ELBOW = 30; //True Zero = faces previous axis
            const double MAX_RANGE_ELBOW = 180;
            const double TO_DEGREES_ELBOW = 0.95; //Placeholder
            const double HOME_POSITION_ELBOW = 40;
            
            //For Wrist Axis
            const double TO_DEGREES_WRIST = 0.3; //Placeholder
            const double MIN_RANGE_WRIST = 80; //True Zero = faces previous axis
            const double MAX_RANGE_WRIST = 270;
            const double HOME_POSITION_WRIST = 90;
            

            TalonSRX Shoulder_Motor, Elbow_Motor, Wrist_Motor;
    };
}