package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * This subsystem represents the intake system on the robot, it will set the
 * motor speed in percent and set pistons to on or off
 */
public class IntakeSubsystem extends SubsystemBase {
    // PS: SillyPandaDog is also Jokorie
    // Initialize Intake Motors
    TalonSRX Intake = new TalonSRX(Constants.MOTOR_ID);

    // take double parameter as the percentage of power given to each motor
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    /**
     * Gets the status of the intake pistons.
     * 
     * @returna boolean true if pistons are on, else false
     */
    public Value intakeGet() {
        return (solenoid1.get());
    }

   
    public void pistonToggle() { 
        solenoid1.toggle();
    }

    /**
     * Gets the percent output speed of the intake motor.
     * 
     * @return the power of the motor control in percent, from -1 to 1 or -100% to
     *         100%
     */
    public double intakeSpeedGet() {
        return Intake.getMotorOutputPercent();

    }

    /**
     * Set the percent output speed of the intake motor.
     * 
     * @param percentSpeed takes in a double value between -1 and 1, representing
     *                     the motor power in percent, or from -100% to 100%
     */
    public void intakeSpeedSet(double percentSpeed) {

        Intake.set(TalonSRXControlMode.PercentOutput, percentSpeed);
    }

}
