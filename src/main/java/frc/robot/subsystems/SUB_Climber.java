// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Climber extends SubsystemBase {
  /** Creates a new Climber. */
  static CANSparkMax climberMotor;
  static DoubleSolenoid cSolenoid;
  static boolean pistonExtended = false;

  public SUB_Climber() {
    climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
    cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   //SmartDashboard.putBoolean("Climber Lock", lockGetPos());
  }

  /**
   * Sets the Climer motor speed
   * 
   * @param speed Can set either a pos or neg speed for the climber arms
   */
  public void speedSet(double speed) {
    climberMotor.set(speed);
  }

  /**
   * Toggle lock and unlock of the climber
   */
 
   public void climberLock(){
    cSolenoid.toggle();
  }

  /**
   * Gets the position of the piston and checks if locked
   * 
   * @param isPistonExtended same value as pistonExtended
   * @return The state of the piston
   */
    public Value lockGet() {
      return (cSolenoid.get());
  
  }

}