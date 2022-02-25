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

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  static CANSparkMax climberMotor;
  static DoubleSolenoid cSolenoid;
  static boolean pistonExtended = false;

  public Climber() {
    climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
<<<<<<< HEAD
    cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
=======
    cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
>>>>>>> 0e4287e (almost down with climber)
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
<<<<<<< HEAD
  public void speedSet(double speed) {
=======
  public static void speedSet(double speed) {
>>>>>>> 0e4287e (almost down with climber)
    climberMotor.set(speed);
  }

  /**
   * Toggle lock and unlock of the climber
   */
<<<<<<< HEAD
 
   public void climberLock(){
=======
  public void climberLock(){
>>>>>>> 0e4287e (almost down with climber)
    cSolenoid.toggle();
  }

  /**
   * Gets the position of the piston and checks if locked
   * 
   * @param isPistonExtended same value as pistonExtended
   * @return The state of the piston
   */
<<<<<<< HEAD
=======
  
>>>>>>> 0e4287e (almost down with climber)
    public Value lockGet() {
      return (cSolenoid.get());
  
  }

}
