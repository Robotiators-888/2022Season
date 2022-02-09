package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.MotorFeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

TalonSRX frontIntake = new TalonSRX(Constants.FRONT_INTAKE_MOTOR_ID);
TalonSRX middleIntake = new TalonSRX(Constants.MIDDLE_INTAKE_MOTOR_ID);

//determines wether ball is coming from the front or back
boolean inFront;
//power of the motor when active
double motorPower = 1.0;
//take double parameter as the percentage of power given to each motor
public void SetMotors(double front, double middle) 
{
    frontIntake.set(TalonSRXControlMode.PercentOutput, front);
    middleIntake.set(TalonSRXControlMode.PercentOutput, middle);
    
}
public void StopAllMotors()
{
 SetMotors(0, 0);
};
public void Intake()
{
    SetMotors(motorPower,0);
    System.out.println("Intaking...");
}

public void Outtake()
{
    SetMotors(0, -motorPower);
    System.out.println("Spitting Out!");
}
public void Index()
{
SetMotors(0, motorPower);
}
public void IntakeSubsytem(boolean ballDetected, boolean isTeamColor)
{
    //ball is detected in front of robot
    while(ballDetected == true)
    {
        Intake();
        while(isTeamColor == true)
        {
            Index();
        }
    }
}
}  

