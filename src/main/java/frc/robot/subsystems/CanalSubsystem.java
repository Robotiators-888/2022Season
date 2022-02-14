// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import frc.robot.Constants;


/**
 * Manages the canal subsystem, runs the front and back belts.
 */
public class CanalSubsystem extends SubsystemBase {
  private TalonSRX front = new TalonSRX(Constants.FRONT_CANAL_ID);
  private TalonSRX back = new TalonSRX(Constants.BACK_CANAL_ID);
  /** Creates a new CanalSubsystem. */
  public CanalSubsystem() {

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  /**
   * Sets the front motor to a certain speed
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedFront(double speed){
     // Front belt, two sets of belts.
     front.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /**
   * Sets the back motor to a certain speed
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedBack(double speed){
    // Back belt, two sets of belts.
    back.set(TalonSRXControlMode.PercentOutput, speed);
  }

}
