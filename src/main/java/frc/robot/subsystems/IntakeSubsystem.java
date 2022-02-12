package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.MotorFeedbackSensor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value; 


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
//PS: SillyPandaDog is also Jokorie
//Initialize Intake Motors
TalonSRX Intake = new TalonSRX(Constants.MOTOR_ID);

//power of the motor when active
double motorPower = 0.0;
//take double parameter as the percentage of power given to each motor
DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
DoubleSolenoid solenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
public boolean intakeGet() {
    if (solenoid1.get().equals(DoubleSolenoid.Value.kForward) && solenoid2.get().equals(DoubleSolenoid.Value.kForward)) {
        return true; 
    } else {
        return false; 
    }
}
public void intakeSet(boolean pistonState) { //up false, down true
    if (intakeGet()) {
        pistonState == true; 
    } else {
        pistonState == false; 
    }
}
public double intakeSpeedGet() {
    motorPower = TalonSRXControlMode.PercentOutput; 
    return motorPower; 
}

public void intakeSpeedSet(double percentSpeed) {
    
    Intake.set(TalonSRXControlMode.PercentOutput, percentSpeed); 
}

}
