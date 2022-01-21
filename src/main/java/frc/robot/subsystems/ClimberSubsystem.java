// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  double ClimberSpeed;
  CANSparkMax ClimberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
 
  public ClimberSubsystem() {}

  public void ClimberUp(){
    ClimberSpeed = Constants.CLIMBER_SPEED;
    ClimberMotor.set(ClimberSpeed);
  }

  public void ClimberDown(){
    ClimberSpeed = (Constants.CLIMBER_SPEED * -1);
    ClimberMotor.set(ClimberSpeed);
  }

  public void ClimberStop(){
    ClimberMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
