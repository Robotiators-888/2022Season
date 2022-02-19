// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import frc.robot.subsystems.BannerSensorSubsystem;

import frc.robot.Constants;

/**
 * Manages the indexing subsystem, moves indexing belts, and detects balls and
 * ball color in the system.
 */
public class IndexSubsystem extends SubsystemBase {

  // Motors
  
  private TalonSRX tower = new TalonSRX(Constants.TOWER_INDEX_ID);
<<<<<<< HEAD
=======

  private BannerSensorSubsystem bannerSensorControl= new BannerSensorSubsystem();
  
  
>>>>>>> f3c6063 (Added banner sensor functionality)

  private BannerSensorSubsystem bannerSensorControl= new BannerSensorSubsystem();


  // Color sensor subsystem
  // private ColorSensorSubsystem colorSensor;

  /** Creates a new ColorSensorSubsystem. */
  public IndexSubsystem() {
    // ColorSensorSubsystem colorSensorArg
    // this.colorSensor = colorSensorArg;
  }

  @Override
  public void periodic() {

  }

  /**
   * Returns if a ball exists at a certain id
   * 
   * @param id is either 1(top spot below shooter) or 2(balls to be stored below).
   * @return true if a ball is detected at the id, false if otherwise.
   * */ 
  
  public boolean getPosition(int id){
    return bannerSensorControl.getValue(id);
  }


  /**
   * Sets the speed of the tower motor to a Constant speed
   */
  public void feed() {
    setSpeedTower(Constants.TOWER_BELT_SPEED);
  }

  /**
   * Sets the speed of the tower motor to zero (stops the motor)
   */
  public void stopFeed() {
    setSpeedTower(0);
  }

  /**
   * Sets the tower motor to a certain speed
   * 
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedTower(double speed) {
    // Top-most belt, move to get it into shooter

      tower.set(TalonSRXControlMode.PercentOutput,speed);

  }

}
