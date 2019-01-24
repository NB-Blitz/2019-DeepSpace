#include "Mecanum.hpp"

using namespace std;

void Blitz::Mecanum::SetMotorDirection(int Motor, int dir)
{
    MotorDirs[Motor] = dir;
}

void Blitz::Mecanum::Initialize(Blitz::Models::MecanumInput *Input)
{
    InputData = Input;
    
    Motors->Motor1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
	Motors->Motor1->SetSensorPhase(true);
    Motors->Motor1->ConfigNominalOutputForward(0, 30);
    Motors->Motor1->ConfigNominalOutputReverse(0, 30);
    Motors->Motor1->ConfigPeakOutputForward(1, 30);
    Motors->Motor1->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor1->Config_kF(0, Blitz::DriveReference::MOTOR1_kF, 30);
    Motors->Motor1->Config_kP(0, Blitz::DriveReference::MOTOR1_kP, 30);
    Motors->Motor1->Config_kI(0, Blitz::DriveReference::MOTOR1_kI, 30);
    Motors->Motor1->Config_kD(0, Blitz::DriveReference::MOTOR1_kD, 30);

    Motors->Motor2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
	Motors->Motor2->SetSensorPhase(true);
    Motors->Motor2->ConfigNominalOutputForward(0, 30);
    Motors->Motor2->ConfigNominalOutputReverse(0, 30);
    Motors->Motor2->ConfigPeakOutputForward(1, 30);
    Motors->Motor2->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor2->Config_kF(0, Blitz::DriveReference::MOTOR2_kF, 30);
    Motors->Motor2->Config_kP(0, Blitz::DriveReference::MOTOR2_kP, 30);
    Motors->Motor2->Config_kI(0, Blitz::DriveReference::MOTOR2_kI, 30);
    Motors->Motor2->Config_kD(0, Blitz::DriveReference::MOTOR2_kD, 30);

    Motors->Motor3->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
	Motors->Motor3->SetSensorPhase(true);
    Motors->Motor3->ConfigNominalOutputForward(0, 30);
    Motors->Motor3->ConfigNominalOutputReverse(0, 30);
    Motors->Motor3->ConfigPeakOutputForward(1, 30);
    Motors->Motor3->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor3->Config_kF(0, Blitz::DriveReference::MOTOR3_kF, 30);
    Motors->Motor3->Config_kP(0, Blitz::DriveReference::MOTOR3_kP, 30);
    Motors->Motor3->Config_kI(0, Blitz::DriveReference::MOTOR3_kI, 30);
    Motors->Motor3->Config_kD(0, Blitz::DriveReference::MOTOR3_kD, 30);

    Motors->Motor4->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
	Motors->Motor4->SetSensorPhase(true);
    Motors->Motor4->ConfigNominalOutputForward(0, 30);
    Motors->Motor4->ConfigNominalOutputReverse(0, 30);
    Motors->Motor4->ConfigPeakOutputForward(1, 30);
    Motors->Motor4->ConfigPeakOutputReverse(-1, 30);
    Motors->Motor4->Config_kF(0, Blitz::DriveReference::MOTOR4_kF, 30);
    Motors->Motor4->Config_kP(0, Blitz::DriveReference::MOTOR4_kP, 30);
    Motors->Motor4->Config_kI(0, Blitz::DriveReference::MOTOR4_kI, 30);
    Motors->Motor4->Config_kD(0, Blitz::DriveReference::MOTOR4_kD, 30);
    
    Motors->Motor1->Set(ControlMode::Velocity, 0);
    Motors->Motor2->Set(ControlMode::Velocity, 0);
    Motors->Motor3->Set(ControlMode::Velocity, 0);
    Motors->Motor4->Set(ControlMode::Velocity, 0);

}

 void Blitz::Mecanum::Drive()
 {
     if(UsePID)
     {
        Motors->Motor1->Set(ControlMode::Velocity, (InputData->XValue + InputData->YValue + InputData->ZValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::CTRE_MILLISECOND_CONVERSION);
        Motors->Motor2->Set(ControlMode::Velocity, (-InputData->XValue + InputData->YValue + InputData->ZValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::CTRE_MILLISECOND_CONVERSION);
        Motors->Motor3->Set(ControlMode::Velocity, (-InputData->XValue + InputData->YValue - InputData->ZValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::CTRE_MILLISECOND_CONVERSION);
        Motors->Motor4->Set(ControlMode::Velocity, (InputData->XValue + InputData->YValue - InputData->ZValue) * Blitz::DriveReference::ENCODER_UNITS_PER_METER / Blitz::DriveReference::CTRE_MILLISECOND_CONVERSION);
    }
    else
    {
        Motors->Motor1->Set(ControlMode::PercentOutput, (InputData->XValue + InputData->YValue + InputData->ZValue));
        Motors->Motor2->Set(ControlMode::PercentOutput, (-InputData->XValue + InputData->YValue + InputData->ZValue));
        Motors->Motor3->Set(ControlMode::PercentOutput, (-InputData->XValue + InputData->YValue - InputData->ZValue));
        Motors->Motor4->Set(ControlMode::PercentOutput, (InputData->XValue + InputData->YValue - InputData->ZValue));
    }
 }

 void Blitz::Mecanum::Close()
 {

 }