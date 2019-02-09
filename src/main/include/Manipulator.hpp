#ifndef SRC_MANIPULATOR_HPP_
#define SRC_MANIPULATOR_HPP_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc
{
    class Manipulator
    {
        private:
            TalonSRX Main_Axis;//, Secondary_Axis;
            DigitalInput Main_Axis_Limit_Switch;//, Secondary_Axis_Limit_Switch; 

            //For Main Axis
            const double TO_DEGREES_MAIN = 20;
            const double DEAD_ZONE_MAIN = 5;
            const double MAX_RANGE_MAIN = 180;
            const double LENGTH_MAIN = 12; //in inches

            /*
            //For Secondary Axis
            const double TO_DEGREES_SECONDARY = 22;
            const double DEAD_ZONE_SECONDARY = 5;
            const double MAX_RANGE_SECONDARY = 180;
            const double LENGTH_SECONDARY = 10; //In inches
            */

            //PID Coefficients for Main
            double PID_P_MAIN = 0.5;
            double PID_I_MAIN = 0; 
            double PID_D_MAIN = 0;
            double PID_F_MAIN = 0;

            /*
            //PID Coefficients for Secondary
            double PID_P_SECONDARY = 0.25;
            double PID_I_SECONDARY = 0; 
            double PID_D_SECONDARY = 0;
            double PID_F_SECONDARY = 0;
            */

        public:
            Manipulator();
            void manipSet(double speed, int axisID); //PercentageOutput (No PID)
            void manipSetPID(double degrees, int axisID); //Position (With PID)
            bool isLimit(int axisID);
            double getDegrees(int axisID); //0 is Main, 1 is Secondary...
            void resetDegrees(int axisID); //See above
            void moveTo(double degrees, int axisID);
            void resetToEncoder(int axisID);
            void initializePID(bool firstTime);
            double getP(int axisID);
            double getI(int axisID);
            double getD(int axisID);
            double getF(int axisID);
            //void updatePIDCoefficients();
            //double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            //void moveToCoordinates(double x, double y); //In inches

    };
}

#endif