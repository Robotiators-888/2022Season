// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.revrobotics.*;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class SUB_Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private DoubleSolenoid cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);
  private AHRS navx; 

  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private SparkMaxLimitSwitch climberLimitSwitch =  climberMotor.getReverseLimitSwitch(Type.kNormallyOpen);

  public SUB_Climber(AHRS navxArgs) {
    this.navx = navxArgs;
    climberMotor.setOpenLoopRampRate(0.5);
    cSolenoid.set(Value.kReverse);
    climberMotor.setIdleMode(IdleMode.kBrake);
    zeroPitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   //SmartDashboard.putBoolean("Climber Lock", lockGetPos());
   SmartDashboard.putNumber("Navx Pitch Val", getPitch());
   SmartDashboard.putNumber("Climber Encoder Val", climberEncoder.getPosition());

   if(climberLimitSwitch.isPressed()==true==true){
     climberEncoder.setPosition(0);
   }
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
 
   public void lockToggle(){
    cSolenoid.toggle();
  }

  public void lockSet(Value input){
    cSolenoid.set(input);
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

  public double getPitch(){
    return navx.getPitch();
  }

  public void zeroPitch(){
    navx.reset();
  }

}
