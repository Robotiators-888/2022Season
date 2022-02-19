// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putBoolean("Climber Lock", lockGetPos());
  }

  /**
   * Sets the Climer motor speed
   * 
   * @param speed Can set either a pos or neg speed for the climber arms
   */
  public static void speedSet(double speed) {
    climberMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /**
   * Toggle lock and unlock of the climber
   */
  public static void toggleLockSet() {
    if (!(lockGetPos(pistonExtended))) {
      cSolenoid.set(Value.kForward);
      pistonExtended = true;
    } else {
      cSolenoid.set(Value.kReverse);
      pistonExtended = false;
    }
  }

  /**
   * Gets the position of the piston and checks if locked
   * 
   * @param isPistonExtended same value as pistonExtended
   * @return The state of the piston
   */
  public static boolean lockGetPos(boolean isPistonExtended) {
    
    return (isPistonExtended);
  }

}
