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
  /** Creates a new CanalSubsystem. */
  public CanalSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
