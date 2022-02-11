// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  static TalonSRX climberMotor;
  static DoubleSolenoid cSolenoid;
  static boolean pistonExtended = false;

  public Climber() {
    climberMotor = new TalonSRX(Constants.CLIMBER_MOTOR_ID);
    cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /**
   * Sets the Climer motor speed to go up
   */
  public static void speedSetUp() {
    climberMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
  }

  /**
   * Sets the Climer motor speed to go down
   */
  public static void speedSetDown() {
    climberMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
  }

  /**
   * Locks the climber in place
   */
  public static void lockSet() {
    if (!(lockGet(pistonExtended))) {
      cSolenoid.set(Value.kForward);
    } else {
      cSolenoid.set(Value.kReverse);
    }
  }

  /**
   * Gets the position of the piston and checks if locked
   * 
   * @param isPistonExtended same value as climberMotor
   * @return The state of the piston
   */
  public static boolean lockGet(boolean isPistonExtended) {

    if (isPistonExtended) {
      pistonExtended = false;
      return false;

    } else {
      pistonExtended = true;
      return true;
    }

  }

}
