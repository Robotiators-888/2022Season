package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax flywheel = new CANSparkMax(Constants.FLYWHEEL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    SparkMaxPIDController PID = flywheel.getPIDController();

    private int manualRPM = 2000;

    public Shooter() {
        this.setPIDF(0.0004, 0.0, 0.0, 0.000288);

    }

    public void periodic() {
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("Manual RPM SetPoint", manualRPM);
    }

    public double getRPM() {
        return flywheel.getEncoder().getVelocity();
    }

    public void setRPM(int rpm) {
        PID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
    
    public void changeManualRPM(int change){
        manualRPM += change;
    }

    public int getManualRPM(){
        return manualRPM;
    }

    public void setPIDF(double P, double I, double D, double F) {
        PID.setP(P);
        PID.setI(I);
        PID.setD(D);
        PID.setFF(F);

    }

    public int distRpm(double dist) {
        return (int) ((dist + 108) / 0.1);

    }

    public void setSpeed(double speed) {
        flywheel.set(speed);
    }
}
