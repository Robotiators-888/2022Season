package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

TalonSRX intakeMotor;
XboxController controller;

//Initialize Piston Solenoids (forward and backward)
DoubleSolenoid intakePistonR = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, Constants.SOLENOID_FRIGHT_ID, Constants.SOLENOID_BRIGHT_ID);
DoubleSolenoid intakePistonL = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, Constants.SOLENOID_FLEFT_ID, Constants.SOLENOID_BLEFT_ID);

//power of the motor when active (percentage)
double motorPower = 1.0;
boolean isExtended = false;

public IntakeSubsystem (int intakeMotor, int controller)
{ 
this.intakeMotor = new TalonSRX(intakeMotor);
this.controller = new XboxController(controller);
}
public void Set()
{
    intakeMotor.set(TalonSRXControlMode.PercentOutput, motorPower);
}
//take double parameter as the percentage of power given to each motor
public void intakeSpeedSet(int speed)
{
    motorPower = speed;
}

public double intakeSpeedGet()
{
    return motorPower;
}
public void intakeSet(boolean extend)
{
    if(extend == true)
    {
    intakePistonL.set(kForward);
    intakePistonR.set(kForward);
    } else 
    {
        intakePistonL.set(kReverse);
        intakePistonR.set(kReverse);
    }
}
public void ToggleIntakeArms()
{
    intakePistonL.toggle();
    intakePistonR.toggle();
}
public void Stop()
{
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
};
public void Intake()
{
    motorPower = 1;
    System.out.println("Intaking...");
}

public void Outtake()
{
    motorPower = -1;
    System.out.println("Spitting Out!");
}

public boolean intakeGet()
{
    return isExtended;
}
public void IntakeSubsytem()
{
    

}
}  

