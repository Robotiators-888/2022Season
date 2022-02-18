package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax flywheel = new CANSparkMax(Constants.FLYWHEEL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    SparkMaxPIDController PID = flywheel.getPIDController();
    Joystick joystick = new Joystick(2);

    public Shooter() {
        this.setPIDF(Constants.P_VALUE, Constants.I_VALUE, Constants.D_VALUE, Constants.F_VALUE);
        flywheel.setIdleMode(IdleMode.kCoast);
        PID.setOutputRange(-1, 1);

    }

    public void periodic() {
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("Shooter %", -(1 - joystick.getRawAxis(3)) / 2);
    }

    public double getRPM() {
        return flywheel.getEncoder().getVelocity();
    }

    public void setRPM(int rpm) {
        PID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
    

    public void setPIDF(double P, double I, double D, double F) {
        PID.setP(P);
        PID.setI(I);
        PID.setD(D);
        PID.setFF(F);

    }
    /**
     * 
     * @param dist
     * @return
     */
    public int distRpm(double dist) {
        return (int) (450 * (Math.sqrt(dist)));

    }

    public void setSpeed(double speed) {
        flywheel.set(speed);
    }
}
