#pragma once

enum armAxis
{ 
  Shoulder_Axis = 0,
  Elbow_Axis = 1,
  Wrist_Axis = 2

};
enum customSpeed
{
  Off = 0
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
            bool manipSetToHome();
            double getRawUnits(int axisID);
            double getDegrees(int axisID, double rawHome); //0 is Main, 1 is Secondary...
            double getAngleForCoordinates(double x, double y, int axisID); //used by next method
            bool moveToRawCounts(double rawShoulder, double rawElbow, double rawWrist);
            void moveToAngles(double shoulderAngle, double elbowAngle, double wristAngle, double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist);
            void moveToCoordinates(double x, double y, double rawHomeShoulder, double rawHomeElbow); //In inches
            bool isPossible(double x, double y);
            void moveToXDegreesBelowParallel(double rawHomeShoulder, double rawHomeElbow, double rawHomeWrist, double x);
            double optimizeAutomaticMovement(double desiredShoulder, double desiredElbow, int axisID);
            double getSpeed(double minSpeed, double maxSpeed, double currentPosition, double desiredPosition, bool isReversed);
            void ResetPosition();
            void MoveManipulatorSpeed(double speed);
            void MoveManipulatorPosition(double diameter);
            void InitializeArm();

            double currentPosition = 0;

            const double HOME_POSITION_SHOULDER = 447;
            const double HOME_POSITION_ELBOW = 414;
            const double HOME_POSITION_WRIST = 394;
            const double UNIVERSAL_SAFE_POSITION_ELBOW = 318;

        private:

            //For Shoulder Axis
            const double LENGTH_SHOULDER = 25;
            const double MIN_RANGE_SHOULDER = 200; //True Zero = faces ground
            const double MAX_RANGE_SHOULDER = 370;
            const double TO_DEGREES_SHOULDER = 1.65; //Placeholder
            
            //For Elbow Axis
            const double LENGTH_ELBOW = 23;
            const double MIN_RANGE_ELBOW = 30; //True Zero = faces previous axis
            const double MAX_RANGE_ELBOW = 180;
            const double TO_DEGREES_ELBOW = 2.8; //Placeholder
/*
    367 Shoulder
    35 Elbow
    227 Wrist
*/
            
            //For Wrist Axis
            const double TO_DEGREES_WRIST = 1.72; //Placeholder
            const double MIN_RANGE_WRIST = 80; //True Zero = faces previous axis
            const double MAX_RANGE_WRIST = 270;

            const double SMALL_GEAR_TEETH = 24;
            const double LARGE_GEAR_TEETH = 57;
            const double PULSES_PER_ROTATION = 180;

            const double PULSES_PER_ANGLE_SMALL_GEAR = ((PULSES_PER_ROTATION/LARGE_GEAR_TEETH) * SMALL_GEAR_TEETH)/360;

            int direction = 1;

            frc::DigitalInput LimitSwitch;
            frc::Counter PositionCounter;
            
            TalonSRX ClawTalon;
            TalonSRX Shoulder_Motor; 
            TalonSRX Elbow_Motor;
            TalonSRX Wrist_Motor;


    };
}