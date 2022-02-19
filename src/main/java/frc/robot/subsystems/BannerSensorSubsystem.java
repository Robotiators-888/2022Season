// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


/**
 * Controls the banner sensors, can fetch values from them.
 */
public class BannerSensorSubsystem extends SubsystemBase {

  private DigitalInput bannerSensor1 = new DigitalInput(Constants.DIO_PORT_0);
  private DigitalInput bannerSensor2 = new DigitalInput(Constants.DIO_PORT_1);

  /** Creates a new BannerSensorSubsystem. */
  public BannerSensorSubsystem() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Returns the value of the banner sensor at a specific abstract id
   * @param id is either 1(top spot below shooter) or 2(balls to be stored below).
   * @return Returns a boolean of a high or low value from the DIO
   */
  public boolean getValue(int id){

    if (id==1){

      return bannerSensor1.get();

    } else if (id==2) {

      return bannerSensor2.get();

    } else {
      System.out.println("Given a bad value. Returning the 2nd sensor's value.");
      return bannerSensor2.get();

    }
  }
  
}
