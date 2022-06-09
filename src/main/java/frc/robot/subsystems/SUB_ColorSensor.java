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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;


/**
 * Manages the color sensors, reads values from the sensors, controls the MUX, and returns String colors from the sensor values.
 */
public class SUB_ColorSensor extends SubsystemBase {

  // I2C Constants
  private I2C.Port rioI2CPort = I2C.Port.kOnboard;
  private I2C.Port navxI2CPort = I2C.Port.kMXP;
  int i2cPortId = 0x70;
  public final int ColorSensorid = 0;

  // Opposition Ball Counter
  private ArrayList<Boolean> ballStack = new ArrayList<Boolean>();

  // Senses colors
  private final ColorSensorV3 frontColorSensor = new ColorSensorV3(rioI2CPort);
  private final ColorSensorV3 backColorSensor = new ColorSensorV3(navxI2CPort);
  // Matches colors
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.562, 0.351, 0.100);
  public Color detectedColor;

  // Alliance Stuff
  private Alliance curAlliance;
  private Alliance oppAlliance;



  /** Creates a new ColorSensorSubsystem. */
  public SUB_ColorSensor() {

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    curAlliance = DriverStation.getAlliance();

    if(curAlliance==Alliance.Red){oppAlliance=Alliance.Blue;}
    else{oppAlliance=Alliance.Red;}

  }

  @Override
  public void periodic() {}

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
   * @return a Alliance color, either Red, Blue, or Inveralid.
   */
  public Alliance colorToAlliance() {
    final double idealRedBlueConfidence = 0.8;
    Alliance colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = Alliance.Blue;
    } else if (match.color == kRedTarget) {
      colorString = Alliance.Red;
    } else {
      colorString = Alliance.Invalid;
    }

    if (match.confidence <= idealRedBlueConfidence) {
     colorString = Alliance.Invalid;
    }

    return colorString;
  }
  /**
   * Reads the sensor for a color value
   * @param newId the id of the color sensor you want to read from
   * @return a string with the color of the value read from the sensor
   */
  public Alliance readSensor(int newId){
    
    switch (newId){
      case 0:
        detectedColor = frontColorSensor.getColor();
        break;
      case 1:
        detectedColor = backColorSensor.getColor();
        break;
    }
    return colorToAlliance();
  }

  public boolean allianceToBool(Alliance all){
    return (all==curAlliance);
  }
  
  public Alliance boolToAlliance(boolean bool){
    if (bool){
      return curAlliance;
    } 
    return oppAlliance;
  }
  public void pushStack(boolean ball){
    ballStack.add(ball);
  }

  public void popStack(){
    ballStack.remove(0);
  }

  public boolean peekStack(){
    return ballStack.get(0);
  }



}
