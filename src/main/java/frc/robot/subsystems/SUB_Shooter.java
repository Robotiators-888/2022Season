package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    CANSparkMax flywheelFollower = new CANSparkMax(Constants.FLYWHEEL_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax flywheel = new CANSparkMax(Constants.FLYWHEEL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    SparkMaxPIDController PID = flywheel.getPIDController();

    private int manualRPM = -1800;

    public SUB_Shooter() {
        this.setPIDF(Constants.P_VALUE, Constants.I_VALUE, Constants.D_VALUE, Constants.F_VALUE);
        flywheel.setIdleMode(IdleMode.kCoast);
        PID.setOutputRange(-1, 1);
        flywheel.enableVoltageCompensation(12);
        flywheelFollower.enableVoltageCompensation(12);
        flywheel.setInverted(true);
        flywheelFollower.follow(flywheel, true);

    }


    public void periodic() {
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("Manual RPM SetPoint", manualRPM);
    }

    /**
     * @return the speed of the flywheel in RPM
     */
    public double getRPM() {
        return flywheel.getEncoder().getVelocity();
    }

    /**
     * Sets the speed in RPM for the flywheel to spin
     * @param rpm int RPM to spin at
     */
    public void setRPM(int rpm) {
        PID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
    
    /**
     * changes the manual rpm setpoint 
     * @param change positive or negative intiger to change the RPM by
     */
    public void changeManualRPM(int change){
        if((this.manualRPM + change) < 0){
        this.manualRPM = this.manualRPM + change;
        }
    }

    /**
     * @return int manual rpm variable 
     */
    public int getManualRPM(){
        return this.manualRPM;
    }

    /**
     * sets the PID values of the shooter control loop
     * @param P double P gain
     * @param I double I gain
     * @param D double D gain
     * @param F double feed forward
     */
    public void setPIDF(double P, double I, double D, double F) {
        PID.setP(P);
        PID.setI(I);
        PID.setD(D);
        PID.setFF(F);

    }

    /**
     * Sets speed for wheel to run at as a precentage of max
     * @param speed double to represent a precentage
     */
    public void setSpeed(double speed) {
        flywheel.set(speed);
    }
}
