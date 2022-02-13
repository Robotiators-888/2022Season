// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import frc.robot.Constants;

/**
 * Manages the indexing subsystem, moves indexing belts, and detects balls and ball color in the system.
 */
public class IndexSubsystem extends SubsystemBase {


  // Motors
  private TalonSRX front = new TalonSRX(Constants.FRONT_INDEX_ID);
  private TalonSRX tower = new TalonSRX(Constants.TOWER_INDEX_ID);
  private TalonSRX back = new TalonSRX(Constants.BACK_INDEX_ID);
  
  // Color sensor subsystem
  //private ColorSensorSubsystem colorSensor;



  /** Creates a new ColorSensorSubsystem. */
  public IndexSubsystem() {
    //ColorSensorSubsystem colorSensorArg
    //this.colorSensor = colorSensorArg;
  }

  @Override
  public void periodic() {

  }



  /** 
   * Returns if a ball exists at a certain id
   * @param id is either 1(top spot below shooter) or 2(balls to be stored below).
   * @return true if a ball is detected at the id, false if otherwise.
   * */ 
  /*
  public boolean getPosition(int id){
    int newId = 0;
    if (id==1){
      // placeholder
      newId = 0;
    } else if (id==2){
      // placeholder
      newId = 1;
    }

    colorSensor.readSensor(newId);
    return !(colorSensor.colorToString().equals("Unknown"));
  }
  /**
   * Returns the color detected at a certain id
   * @param id is either 1(top spot below shooter) or 2(balls to be stored below).
   * @return the color in the form of a string, Red, Black, Blue, or Unknown.
   */

  /*
  public String getColor(int id){
    int newId = 0;
    if (id==1){
      // placeholder
      newId = 0;
    } else if (id==2){
      // placeholder
      newId = 1;
    }
    
    colorSensor.readSensor(newId);
    return colorSensor.colorToString();
  }
  */

  /**
   * Sets the speed of the tower motor to a Constant speed
   */
  public void feed(){
    setSpeedTower(Constants.TOWER_BELT_SPEED);
  }
  
  /**
   * Sets the speed of the tower motor to zero (stops the motor)
   */
  public void stopFeed(){
    setSpeedTower(0);
  }

  public void indexOut(){
    // when it gets stuck
    setSpeedBack(0.75);
  }

  /**
   * Sets the front motor to a certain speed
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedFront(double speed){
     // Front belt, two sets of belts.
     front.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /**
   * Sets the back motor to a certain speed
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedBack(double speed){
    // Back belt, two sets of belts.
    back.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /**
   * Sets the tower motor to a certain speed
   * @param speed is how fast you want it to go in percentages
   */
  public void setSpeedTower(double speed){
    // Top-most belt, move to get it into shooter
    tower.set(TalonSRXControlMode.PercentOutput,speed);
  }

}
