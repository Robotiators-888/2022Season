// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the indexing subsystem, moves indexing belts, and detects balls and
 * ball color in the system.
 */
public class IndexSubsystem extends SubsystemBase {

  // Motors

  private CANSparkMax tower = new CANSparkMax(Constants.TOWER_INDEX_ID,CANSparkMaxLowLevel.MotorType.kBrushless);
  private DigitalInput bannerSensor1 = new DigitalInput(Constants.DIO_PORT_0);
  private DigitalInput bannerSensor2 = new DigitalInput(Constants.DIO_PORT_1);



  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    tower.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Banner sensor", readTopBanner());
    SmartDashboard.putBoolean("Bottom Banner sensor", readBottomBanner());
  }

  /**
   * readTopBanner returns if the top banner sensor detects a ball or not with a
   * boolean.
   */
  public boolean readTopBanner() {
    // Add a NOT to account for the banner sensor returning false if the ball is there
    return !bannerSensor1.get();

  }

  /**
   * readBottomBanner returns if the bottom banner sensor detects a ball or not
   * with a boolean.
   */
  public boolean readBottomBanner() {
    // Add a NOT to account for the banner sensor returning false if the ball is there
    return !bannerSensor2.get();
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

    tower.set(speed);

  }

}