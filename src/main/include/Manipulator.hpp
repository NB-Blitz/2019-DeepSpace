/*

*/



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
            const double MAX_RANGE_MAIN = 270;
            const double LENGTH_MAIN = 9.5; //in inches
            const double ENCODER_COUNTS_PER_ROTATION_MAIN = 7800;
            const double TO_DEGREES_MAIN = ENCODER_COUNTS_PER_ROTATION_MAIN / 360;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_MAIN = 80; //True Zero = faces parallel to ground and faces forward

            //For Secondary Axis
            const double MAX_RANGE_SECONDARY = 270;
            const double LENGTH_SECONDARY = 4.5; //In inches
            const double ENCODER_COUNTS_PER_ROTATION_SECONDARY = 9000;
            const double TO_DEGREES_SECONDARY = ENCODER_COUNTS_PER_ROTATION_SECONDARY / 360;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SECONDARY = 90; //True Zero = faces previous axis

            /*
            //For Wrist Axis
            const double TO_DEGREES_WRIST = 12;
            const double MAX_RANGE_WRIST = 300;
            const double LENGTH_WRIST = 5; //In inches
            const double ENCODER_COUNTS_PER_ROTATION_WRIST = 3649;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_WRIST = 30; //True Zero = faces previous axis
            */


           /*
                CAUTION!
                Setting motors to move to a degree lower than DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO constant will cause unexpected behavior 
           */

            //PID Coefficients for Main
            double PID_P_MAIN = 0.5;
            double PID_I_MAIN = 0; 
            double PID_D_MAIN = 0;
            double PID_F_MAIN = 0;

            //PID Coefficients for Secondary
            double PID_P_SECONDARY = 0.25;
            double PID_I_SECONDARY = 0; 
            double PID_D_SECONDARY = 0;
            double PID_F_SECONDARY = 0;

           /*
            //PID Coefficients for Wrist
            double PID_P_WRIST = 0.25;
            double PID_I_WRIST = 0; 
            double PID_D_WRIST = 0;
            double PID_F_WRIST = 0;
            */

           TalonSRX Shoulder_Motor, Elbow_Motor;// Wrist_Motor;
           DigitalInput Shoulder_Motor_Limit_Switch, Elbow_Motor_Limit_Switch;//, Wrist_Motor_Limit_Switch; 

        public:
            Manipulator();
            void manipSet(double speed, int axisID, bool areLimits = true, double rawHome = 0); //PercentageOutput (No PID)
            void manipSetToDegrees(double degrees, int axisID, bool areLimits = true, double rawHome = 0);
            bool isLimit(int axisID);
            double getRawUnits(int axisID);
            double getDegrees(int axisID, double rawHome = 0); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            bool resetToEncoder(int axisID); //bool returns if the motor and its encoder is reset
            void initializePID(bool firstTime);
            double getP(int axisID);
            double getI(int axisID);
            double getD(int axisID);
            double getF(int axisID);
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            void moveToCoordinates(double x, double y, bool areLimits = true, double rawHomeShoulder = 0, double rawHomeElbow = 0); //In inches
            bool isPossible(double x, double y);
            //double getAngleForParallel(double x, double y); 
            //void moveToParallel(double x, double y);

    };
}

#endif