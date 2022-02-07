package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

TalonSRX frontIntake = new TalonSRX(ControlMode.kforward, Constants.FRONT_INTAKE_MOTOR_ID);


public void IntakeSubsystem()
{

}
}