#include "Tank.hpp"

void Blitz::Tank::SetMotorDirection(int Motor, int dir)
{
    MotorDirs[Motor] = dir;
}

void Blitz::Tank::TuneF(int MotorID, double FGain)
{
    switch(MotorID)
    {
        case 1 :
            Motors->Motor1->Config_kF(0, FGain, 30);
            break;
        case 2 :
            Motors->Motor2->Config_kF(0, FGain, 30);
            break;
        case 3 :
            Motors->Motor3->Config_kF(0, FGain, 30);
            break;
        case 4 :
            Motors->Motor4->Config_kF(0, FGain, 30);
            break;
        
    }
}

void Blitz::Tank::TuneP(int MotorID, double PGain)
{
    switch(MotorID)
    {
        case 1 :
            Motors->Motor1->Config_kP(0, PGain, 30);
            break;
        case 2 :
            Motors->Motor2->Config_kP(0, PGain, 30);
            break;
        case 3 :
            Motors->Motor3->Config_kP(0, PGain, 30);
            break;
        case 4 :
            Motors->Motor4->Config_kP(0, PGain, 30);
            break;
        
    }

}

void Blitz::Tank::TuneI(int MotorID, double IGain)
{
    switch(MotorID)
    {
        case 1 :
            Motors->Motor1->Config_kI(0, IGain, 30);
            break;
        case 2 :
            Motors->Motor2->Config_kI(0, IGain, 30);
            break;
        case 3 :
            Motors->Motor3->Config_kI(0, IGain, 30);
            break;
        case 4 :
            Motors->Motor4->Config_kI(0, IGain, 30);
            break;
        
    }

}

void Blitz::Tank::TuneD(int MotorID, double DGain)
{
    switch(MotorID)
    {
        case 1 :
            Motors->Motor1->Config_kD(0, DGain, 30);
            break;
        case 2 :
            Motors->Motor2->Config_kD(0, DGain, 30);
            break;
        case 3 :
            Motors->Motor3->Config_kD(0, DGain, 30);
            break;
        case 4 :
            Motors->Motor4->Config_kD(0, DGain, 30);
            break;
        
    }
}

double Blitz::Tank::GetMotorOutput(int MotorID)
{
    return motorValues[MotorID - 1];
}

void Blitz::Tank::Initialize(Blitz::Models::TankInput *Input)
{
    InputData = Input;

    Motors->Motor1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 30);
	Motors->Motor1->SetSensorPhase(true);
    Motors->Motor1->ConfigNominalOutputForward(0, 30);
    Motors->Motor1->ConfigNominalOutputReverse(0, 30);
    Motors->Motor1->ConfigPeakOutputForward(1, 30);
    Motors->Motor1->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor1->Config_kF(0, Blitz::DriveReference::MOTOR1_kF, 30);
    Motors->Motor1->Config_kP(0, Blitz::DriveReference::MOTOR1_kP, 30);
    Motors->Motor1->Config_kI(0, Blitz::DriveReference::MOTOR1_kI, 30);
    Motors->Motor1->Config_kD(0, Blitz::DriveReference::MOTOR1_kD, 30);

    Motors->Motor2->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 30);
	Motors->Motor2->SetSensorPhase(true);
    Motors->Motor2->ConfigNominalOutputForward(0, 30);
    Motors->Motor2->ConfigNominalOutputReverse(0, 30);
    Motors->Motor2->ConfigPeakOutputForward(1, 30);
    Motors->Motor2->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor2->Config_kF(0, Blitz::DriveReference::MOTOR2_kF, 30);
    Motors->Motor2->Config_kP(0, Blitz::DriveReference::MOTOR2_kP, 30);
    Motors->Motor2->Config_kI(0, Blitz::DriveReference::MOTOR2_kI, 30);
    Motors->Motor2->Config_kD(0, Blitz::DriveReference::MOTOR2_kD, 30);

    Motors->Motor3->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 30);
	Motors->Motor3->SetSensorPhase(true);
    Motors->Motor3->ConfigNominalOutputForward(0, 30);
    Motors->Motor3->ConfigNominalOutputReverse(0, 30);
    Motors->Motor3->ConfigPeakOutputForward(1, 30);
    Motors->Motor3->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor3->Config_kF(0, Blitz::DriveReference::MOTOR3_kF, 30);
    Motors->Motor3->Config_kP(0, Blitz::DriveReference::MOTOR3_kP, 30);
    Motors->Motor3->Config_kI(0, Blitz::DriveReference::MOTOR3_kI, 30);
    Motors->Motor3->Config_kD(0, Blitz::DriveReference::MOTOR3_kD, 30);

    Motors->Motor4->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 30);
	Motors->Motor4->SetSensorPhase(true);
    Motors->Motor4->ConfigNominalOutputForward(0, 30);
    Motors->Motor4->ConfigNominalOutputReverse(0, 30);
    Motors->Motor4->ConfigPeakOutputForward(1, 30);
    Motors->Motor4->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor4->Config_kF(0, Blitz::DriveReference::MOTOR4_kF, 30);
    Motors->Motor4->Config_kP(0, Blitz::DriveReference::MOTOR4_kP, 30);
    Motors->Motor4->Config_kI(0, Blitz::DriveReference::MOTOR4_kI, 30);
    Motors->Motor4->Config_kD(0, Blitz::DriveReference::MOTOR4_kD, 30);

    Motors->Motor1->Set(ControlMode::PercentOutput, 0);
    Motors->Motor2->Set(ControlMode::PercentOutput, 0);
    Motors->Motor3->Set(ControlMode::PercentOutput, 0);
    Motors->Motor4->Set(ControlMode::PercentOutput, 0);

}

void Blitz::Tank::Run()
{

    if(UsePID)
    {
        motorValues[0] = (InputData->LeftValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::SECOND_TO_HUNDERD_MILLISECOND_CONVERSION;
        motorValues[1] = (InputData->LeftValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::SECOND_TO_HUNDERD_MILLISECOND_CONVERSION;
        motorValues[2] = (InputData->RightValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::SECOND_TO_HUNDERD_MILLISECOND_CONVERSION;
        motorValues[3] = (InputData->RightValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::SECOND_TO_HUNDERD_MILLISECOND_CONVERSION;

        double maxMagnitude = 0;

        for (double checkValue : motorValues)
	    {
            checkValue = std::fabs(checkValue);

            if (maxMagnitude < checkValue)
            {
                maxMagnitude = checkValue;
            }
        }

        if (maxMagnitude > Blitz::DriveReference::MAX_SPEED_COUNTS_PER_HUNDRED_MILLISECONDS)
        {
            for (double checkValue : motorValues)
            {
                checkValue = checkValue / maxMagnitude;
            }
        }

        Motors->Motor1->Set(ControlMode::Velocity, motorValues[0] * MotorDirs[0]);
        Motors->Motor3->Set(ControlMode::Velocity, motorValues[1] * MotorDirs[1]);
        Motors->Motor2->Set(ControlMode::Velocity, motorValues[2] * MotorDirs[2]);
        Motors->Motor4->Set(ControlMode::Velocity, motorValues[3] * MotorDirs[3]);
    }
    else
    {
        motorValues[0] = InputData->LeftValue / Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        motorValues[1] = InputData->LeftValue / Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        motorValues[2] = InputData->RightValue / Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;
        motorValues[3] = InputData->RightValue / Blitz::DriveReference::MAX_SPEED_METERS_PER_SECOND;

        double maxMagnitude = 0;

        for (double checkValue : motorValues)
	    {
            checkValue = std::fabs(checkValue);

            if (maxMagnitude < checkValue)
            {
                maxMagnitude = checkValue;
            }
        }

        if (maxMagnitude > 1)
        {
            for (int i = 0; i < 4; i++)
            {
                motorValues[i] = motorValues[i] / maxMagnitude;
            }
        }

        Motors->Motor1->Set(ControlMode::PercentOutput, motorValues[0] * MotorDirs[0]);
        Motors->Motor3->Set(ControlMode::PercentOutput, motorValues[1] * MotorDirs[1]);
        Motors->Motor2->Set(ControlMode::PercentOutput, motorValues[2] * MotorDirs[2]);
        Motors->Motor4->Set(ControlMode::PercentOutput, motorValues[3] * MotorDirs[3]);
 
    }
}

void Blitz::Tank::Close()
{

}