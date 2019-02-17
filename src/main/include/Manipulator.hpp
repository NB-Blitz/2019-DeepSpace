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

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <BlitzLib/BlitzLib.hpp>

namespace Blitz
{
    class Manipulator
    {
        public:

            Manipulator();

            void ResetPosition();
            void MoveManipulatorSpeed(double speed);
            void MoveManipulatorPosition(double diameter);

            void manipSet(double speed, int axisID, double rawHome); //PercentageOutput (No PID)
            void manipSetToDegrees(double degrees, int axisID, double rawHome);
            double getRawUnits(int axisID);
            double getDegrees(int axisID, double rawHome); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            void moveToCoordinates(double x, double y, double rawHomeShoulder, double rawHomeElbow); //In inches
            bool isPossible(double x, double y);
            double getAngleForParallel(double x, double y); 
            void moveToParallel(double x, double y, double rawHomeWrist);

            double currentPosition = 0;

        private:
            
            const double SMALL_GEAR_TEETH = 24;
            const double LARGE_GEAR_TEETH = 57;
            const double PULSES_PER_ROTATION = 180;

            const double PULSES_PER_ANGLE_SMALL_GEAR = ((PULSES_PER_ROTATION/LARGE_GEAR_TEETH) * SMALL_GEAR_TEETH)/360;

            const double LENGTH_SHOULDER = 25;
            const double MAX_RANGE_SHOULDER = 270;
            const double COUNTS_PER_QUARTER_SHOULDER = 80; //Placeholder
            const double TO_DEGREES_SHOULDER = COUNTS_PER_QUARTER_SHOULDER / 90;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER = 90; //True Zero = faces ground

            //For Secondary Axis
            const double LENGTH_ELBOW = 23;
            const double MAX_RANGE_ELBOW = 270;
            const double COUNTS_PER_QUARTER_ELBOW = 80; //Placeholder
            const double TO_DEGREES_ELBOW = COUNTS_PER_QUARTER_ELBOW / 90;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_ELBOW = 90; //True Zero = faces previous axis
            
            //For Wrist Axis
            const double MAX_RANGE_WRIST = 260;
            const double COUNTS_PER_QUARTER_WRIST = 80; //for 90 degrees, this is how many counts I get
            const double TO_DEGREES_WRIST = COUNTS_PER_QUARTER_WRIST / 90;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST = 80; //True Zero = faces previous axis

            int direction = 1;

            frc::DigitalInput LimitSwitch;
            frc::Counter PositionCounter;
            TalonSRX ClawTalon;
            TalonSRX Shoulder_Motor, Elbow_Motor, Wrist_Motor;
    };
}