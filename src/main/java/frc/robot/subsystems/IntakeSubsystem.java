// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new InakeSubsystem. */
CANSparkMax IntakeUpperTowerMotor = new CANSparkMax(Constants.INTAKE_UPPER_BELTS_MOTOR_ID, MotorType.kBrushless);
CANSparkMax IntakeLowerTowerMotor = new CANSparkMax(Constants.INTAKE_LOWER_BELTS_MOTOR_ID, MotorType.kBrushless);
CANSparkMax IntakeArmMotors = new CANSparkMax(Constants.INTAKE_ARMS_MOTOR_ID, MotorType.kBrushless);
double IntakeSpeed = Constants.INTAKE_SPEED;

  public IntakeSubsystem() {}


  public void IntakeIn(){
    IntakeArmMotors.set(IntakeSpeed);
    IntakeLowerTowerMotor.set(IntakeSpeed);
    IntakeUpperTowerMotor.set(IntakeSpeed);
  }

  public void IntakeOut(){
    IntakeArmMotors.set(IntakeSpeed * -1);
    IntakeLowerTowerMotor.set(IntakeSpeed * -1);
    IntakeUpperTowerMotor.set(IntakeSpeed * -1);
  }

  public void IntakeStop(){
    IntakeArmMotors.set(0.0);
    IntakeLowerTowerMotor.set(0.0);
    IntakeUpperTowerMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
