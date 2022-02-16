package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
    SparkMaxPIDController PID;
    CANSparkMax indexer;

    public Index() {

        indexer = new CANSparkMax(Constants.BACK_CANAL_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        PID = indexer.getPIDController();

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

    public void setSpeed(double speed) {
        indexer.set(speed);
    }

}
