#include "Manipulator.hpp"
#include "Blitz_Joystick.hpp"

class Robot: public frc::SampleRobot
{
 	frc::Manipulator Manip;
	frc::Blitz_Joystick Blitz_Joy;

public:
	Robot():
		Manip(),
		Blitz_Joy()
	{

	}

	void OperatorControl()
	{
		double degreePosition = 0; //Position of Main_Axis as controlled by Joystick
		const double MAXIMUM_RANGE = 100; //Maximum Range of degreePosition, which controls Main_Axis
  		while (IsOperatorControl() && IsEnabled()) 
  		{
			if (Blitz_Joy.getButton(1)) //This button overrides normal behavior
			{
				Manip.reset(); //Resets Manipulator to original position
				degreePosition = 0; //Resets desired Manipulator Position
			}
			else //Normal Behavior
			{
				//Joystick degreePosition is modified
				if (Blitz_Joy.getJoystick() == 1 && degreePosition < MAXIMUM_RANGE)
				{
					degreePosition += 0.15; //Translates to approximately 30 degrees per second
				}
				else if (Blitz_Joy.getJoystick() == -1 && degreePosition > 0)
				{
					degreePosition -= 0.15; //Translates to approximately -30 degrees per second
				}
				Manip.moveTo(degreePosition); //Manipulator moves to accommidate (Takes into account DEAD_ZONE = 5 degrees)
			}

			//Displays desirable information to Smart Dashboard
			frc::SmartDashboard::PutNumber("Manipulator - Desired Degrees", degreePosition);
			frc::SmartDashboard::PutNumber("Manipulator - Actual Degrees", Manip.getDegrees());
			frc::SmartDashboard::PutBoolean("Is in Reset State?", Blitz_Joy.getButton(1));

    		frc::Wait(0.005);
  		}
	}
};
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif