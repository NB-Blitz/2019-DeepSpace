#ifndef SRC_MANIPULATOR_HPP_
#define SRC_MANIPULATOR_HPP_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc
{
    class Manipulator
    {
        private:
            

            //For Main Axis
            const double TO_DEGREES_MAIN = 20;
            const double MAX_RANGE_MAIN = 270;
            const double LENGTH_MAIN = 9; //in inches
            const double ENCODER_COUNTS_PER_ROTATION_MAIN = 3649;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_MAIN = 90; //True Zero = faces parallel to ground and faces forward

            //For Secondary Axis
            const double TO_DEGREES_SECONDARY = 22;
            const double MAX_RANGE_SECONDARY = 330;
            const double LENGTH_SECONDARY = 4.5; //In inches
            const double ENCODER_COUNTS_PER_ROTATION_SECONDARY = 3649;
            const double DEGREES_BETWEEN_LIMIT_AND_TRUE_ZERO_SECONDARY = 30; //True Zero = faces previous axis

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

        public:
            TalonSRX Main_Axis, Secondary_Axis;// Wrist_Axis;
            DigitalInput Main_Axis_Limit_Switch, Secondary_Axis_Limit_Switch;//, Wrist_Axis_Limit_Switch; 
            Manipulator();
            void manipSet(double speed, int axisID); //PercentageOutput (No PID)
            void manipSetPID(double degrees, int axisID); //Position (With PID)
            bool isLimit(int axisID);
            double getDegrees(int axisID); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            void resetToEncoder(int axisID);
            void initializePID(bool firstTime);
            double getP(int axisID);
            double getI(int axisID);
            double getD(int axisID);
            double getF(int axisID);
            //void updatePIDCoefficients();
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            void moveToCoordinates(double x, double y); //In inches
            //double getAngleForParallel(double x, double y); 
            //void moveToParallel(double x, double y);

    };
}

#endif