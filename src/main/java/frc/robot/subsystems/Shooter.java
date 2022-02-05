package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax flywheel  = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);;
    SparkMaxPIDController PID = flywheel.getPIDController();

    public Shooter() {
        this.setPIDF(0.0004, 0.0, 0.0, 0.000288);
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
    public int distRpm(double dist){
        return(int)((dist + 108)/0.1);

    }

    public void setSpeed(double speed) {
        flywheel.set(speed);
    }
}