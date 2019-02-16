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
            double PID_P_SHOULDER = 0.5;
            double PID_I_SHOULDER = 0; 
            double PID_D_SHOULDER = 0;
            double PID_F_SHOULDER = 0;

            //PID Coefficients for Secondary
            double PID_P_ELBOW = 0.25;
            double PID_I_ELBOW = 0; 
            double PID_D_ELBOW = 0;
            double PID_F_ELBOW = 0;

            /*
            //PID Coefficients for Wrist
            double PID_P_WRIST = 0.25;
            double PID_I_WRIST = 0; 
            double PID_D_WRIST = 0;
            double PID_F_WRIST = 0;
            */
           
            //Dimensions of robot (for isPossible frc rules)
            /*
            
            */

            TalonSRX Shoulder_Motor, Elbow_Motor;// Wrist_Motor;
            DigitalInput Shoulder_Motor_Limit_Switch, Elbow_Motor_Limit_Switch;//, Wrist_Motor_Limit_Switch; 
            //AnalogPotentiometer Shoulder_Pot, Elbow_Pot, Wrist_Pot;
            //const double SHOULDER_POT_OFFSET = 0;
            //const double ELBOW_POT_OFFSET = 0;
            //const double WRIST_POT_OFFSET = 0;
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