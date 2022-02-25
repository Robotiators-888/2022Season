// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the indexing subsystem, moves indexing belts, and detects balls and
 * ball color in the system.
 */
public class IndexSubsystem extends SubsystemBase {

  // Motors

  private TalonSRX tower = new TalonSRX(Constants.TOWER_INDEX_ID);
  private DigitalInput bannerSensor1 = new DigitalInput(Constants.DIO_PORT_0);
  private DigitalInput bannerSensor2 = new DigitalInput(Constants.DIO_PORT_1);

  public enum States {
    ONE_BALL_TOP,
    ONE_BALL_BOTTOM,
    TWO_BALL,
    ZERO_BALL,
  }

  public States currentState;






  // Color sensor subsystem
  // private ColorSensorSubsystem colorSensor;

  /** Creates a new ColorSensorSubsystem. */
  public IndexSubsystem() {
    // ColorSensorSubsystem colorSensorArg
    // this.colorSensor = colorSensorArg;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Banner sensor", readTopBanner());
    SmartDashboard.putBoolean("Bottom Banner sensor", readBottomBanner());

    // Updates the banner states, only used for the calling of intake-canal-index
    // methods
    // Separate states are used to handle how the intake-canal-index methods work
    if (readTopBanner()) {

      if (readBottomBanner()) {
        currentState = States.TWO_BALL;
      } else {
        currentState = States.ONE_BALL_TOP;
      }

    } else if (readBottomBanner()) {
      currentState = States.ONE_BALL_BOTTOM;
    } else {
      currentState = States.ZERO_BALL;
    }

  }

  /**
   * readTopBanner returns if the top banner sensor detects a ball or not with a
   * boolean.
   */
  public boolean readTopBanner() {
    return !bannerSensor1.get();

  }

  /**
   * readBottomBanner returns if the bottom banner sensor detects a ball or not
   * with a boolean.
   */
  public boolean readBottomBanner() {
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

    tower.set(TalonSRXControlMode.PercentOutput, speed);

  }

}
