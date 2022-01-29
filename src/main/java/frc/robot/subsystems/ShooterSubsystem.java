// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  double ShooterSpeed;
  CANSparkMax ShooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
  

  
  public ShooterSubsystem() {

  }


  public void elevatorUp(){
    ShooterSpeed = Constants.SHOOTER_SPEED;
    ShooterMotor.set(ShooterSpeed);
  }
  
  public void ShooterStop(){
  ShooterMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
