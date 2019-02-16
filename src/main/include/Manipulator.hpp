#ifndef SRC_MANIPULATOR_HPP_
#define SRC_MANIPULATOR_HPP_

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

namespace frc
{
    class Manipulator
    {
        private:
            //For Main Axis
            const double MAX_RANGE_SHOULDER = 270;
            const double LENGTH_SHOULDER = 9.5; //in inches
            const double ENCODER_COUNTS_PER_ROTATION_SHOULDER = 7500;
            const double TO_DEGREES_SHOULDER = ENCODER_COUNTS_PER_ROTATION_SHOULDER / 360;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SHOULDER = 90; //True Zero = faces parallel to ground and faces forward

            //For Secondary Axis
            const double MAX_RANGE_ELBOW = 270;
            const double LENGTH_ELBOW = 4.5; //In inches
            const double ENCODER_COUNTS_PER_ROTATION_ELBOW = 8200;
            const double TO_DEGREES_ELBOW = ENCODER_COUNTS_PER_ROTATION_ELBOW / 360;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_ELBOW = 90; //True Zero = faces previous axis

            //For Wrist Axis
            const double MAX_RANGE_WRIST = 260;
            const double COUNTS_PER_QUARTER = 80; //for 90 degrees, this is how many counts I get
            const double TO_DEGREES_WRIST = COUNTS_PER_QUARTER / 90;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST = 80; //True Zero = faces previous axis


           /*
                CAUTION!
                Setting motors to move to a degree lower than DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO constant will cause unexpected behavior 
           */

            //Dimensions of robot (for isPossible frc rules)
            /*
            
            */

            TalonSRX Shoulder_Motor, Elbow_Motor, Wrist_Motor;
            DigitalInput Shoulder_Motor_Limit_Switch, Elbow_Motor_Limit_Switch;

        public:
            Manipulator();
            void manipSet(double speed, int axisID, bool areLimits = true, double rawHome = 0); //PercentageOutput (No PID)
            void manipSetToDegrees(double degrees, int axisID, bool areLimits = true, double rawHome = 0);
            bool isLimit(int axisID);
            double getRawUnits(int axisID);
            double getDegrees(int axisID, double rawHome = 0); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            bool resetToEncoder(int axisID); //bool returns if the motor and its encoder is reset
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            void moveToCoordinates(double x, double y, bool areLimits = true, double rawHomeShoulder = 0, double rawHomeElbow = 0); //In inches
            bool isPossible(double x, double y);
            double getAngleForParallel(double x, double y); 
            void moveToParallel(double x, double y, bool areLimits = false, double rawHomeWrist = 0);

    };
}

#endif