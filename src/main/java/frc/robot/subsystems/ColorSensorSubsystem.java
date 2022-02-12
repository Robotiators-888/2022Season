// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


/**
 * Manages the color sensors, reads values from the sensors, controls the MUX, and returns String colors from the sensor values.
 */
public class ColorSensorSubsystem extends SubsystemBase {

  // I2C Constants
  private I2C.Port i2cPort = I2C.Port.kOnboard;
  int i2cPortId = 0x70;
  public final int ColorSensorid = 0;

  // Senses colors
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  // Matches colors
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.562, 0.351, 0.100);
  private final Color kBlackTarget = new Color(0.252, 0.483, 0.263);
  public Color detectedColor;

    /**
   * Switches the input channel on the MUX switch over I2C
   * @param newMuxPort  the id of the mux chanel to switch the mux to for all future I2C input
   */
  private void MuxChangeI2cPort(int newMuxPort) {
    I2C I2CObject = new I2C(i2cPort,i2cPortId);
    int i2cPortId = 0x70; // MUX I2C address
    // and you simply write a single byte with the desired multiplexed output
    // number to that port
    boolean failed = I2CObject.write(i2cPortId, newMuxPort);
    if (failed) {
      System.out.println("Failed to write to MUX over I2C");
    }
    I2CObject.close();
  }

  /** Creates a new ColorSensorSubsystem. */
  public ColorSensorSubsystem() {

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kBlackTarget);
    colorMatcher.addColorMatch(kRedTarget);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // NOTE: change ColorSensorid to change which color sensor is used
    // MuxChangeI2cPort(i2cPort,ColorSensorid);


  }

 /**
   * Grabs the RGB values from the detected color
   * @return returns a double array with 0: R, 1:B, 2:G.
   */
  public double[] findRGB() {
    double[] RGBArray = new double[3];
    RGBArray[0] = detectedColor.red;
    RGBArray[1] = detectedColor.blue;
    RGBArray[2] = detectedColor.green;

    return RGBArray;
  }


  /**
   * Grabs the color detected from the color sensor at the current I2C port
   * @return a string with the color, either Red, Black, Blue, or Unknown.
   */
  public String colorToString() {
    final double idealRedBlueConfidence = 0.95;
    final double idealBlackConfidence = 0.98;
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kBlackTarget) {
      colorString = "Black";
    } else {
      colorString = "Unknown";
    }

    if (colorString.equals("Black") && match.confidence <= idealBlackConfidence) {
      colorString = "Unknown";
    } else if (colorString.equals("Red") && match.confidence <= idealRedBlueConfidence) {
      colorString = "Unknown";
    } else if (colorString.equals("Blue") && match.confidence <= idealRedBlueConfidence) {
      colorString = "Unknown";
    }

    return colorString;
  }
  /**
   * Reads the sensor for a color value
   * @param newId the id of the color sensor you want to read from
   */
  public void readSensor(int newId){
    MuxChangeI2cPort(newId);
    detectedColor = colorSensor.getColor();
  }
}
