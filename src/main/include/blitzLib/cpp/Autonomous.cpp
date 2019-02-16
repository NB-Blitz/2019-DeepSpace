#include "Autonomous.hpp"

using namespace Blitz;

Blitz::Models::MecanumInput Blitz::Autonomous::Run(double Yaw)
{
    SmartDashboard::PutNumber("Seconds", seconds.Get());
    if(seconds.Get() <= stageOne)
    {
      //Go forwards
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }

    else if (seconds.Get() <= stageTwo)
    {
      if (direction == true)
      {
        //Diagonal right
        MecanumInput.XValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.ZValue = 0;
      }
      
      else
      {
        //Diagonal left
        MecanumInput.XValue = -Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.ZValue = 0;
      }
    }

    else if (seconds.Get() <= stageThree)
    {
      //Go forwards
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }
   
    else if(seconds.Get() <= stageFour)
    {
      //Place the hatch panel/cover on the hatch during this time
      seconds.Stop();
      MecanumInput.XValue = 0;
      MecanumInput.YValue = 0;
      MecanumInput.ZValue = 0;
      seconds.Start();
    }

    else if (seconds.Get() <= stageFive)
    {
      if (direction == true)
      {
        //Go straight right
        MecanumInput.XValue = -Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }

      else
      {
        //Go straight left
        MecanumInput.XValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }
      Yaw = 0; 
    }
    
    else if (seconds.Get() <= stageSix)
    {
      //Turn 180 degrees
      seconds.Stop();
      frc::SmartDashboard::PutNumber("Angle", Yaw);
      if (Yaw >= -15 && Yaw < turnAngle)
      {
        MecanumInput.XValue = 0;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = ((turnAngle - Yaw) * Speed * (1/turnAngle) - Speed / 2 + 0.5) * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      }
      else
      {
        seconds.Start();
        MecanumInput.XValue = 0;
        MecanumInput.YValue = 0;
        MecanumInput.ZValue = 0;
      }

    }
    
    else if (seconds.Get() <= stageSeven)
    {
      //Move forward
      MecanumInput.XValue = 0;
      MecanumInput.YValue = Speed * Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
      MecanumInput.ZValue = 0;
    }
    
    else 
    {
      //Grab the hatch from the wall during this stopped time. 
      seconds.Stop();
      MecanumInput.XValue = 0;
      MecanumInput.YValue = 0;
      MecanumInput.ZValue = 0;
    }
    return MecanumInput;
}

