package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  
    TalonSRX indexer;

    public Index(){

        indexer = new TalonSRX(Constants.BACK_CANAL_ID);
       

    }
    

    public void setSpeed(double speed) {
        indexer.set(TalonSRXControlMode.PercentOutput, speed);
    }

}
