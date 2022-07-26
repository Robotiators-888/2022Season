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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants;


/**
 * Manages the color sensors, reads values from the sensors, controls the MUX, and returns String colors from the sensor values.
 */
public class SUB_ColorSensor extends SubsystemBase {

  // I2C Constants
  private I2C.Port rioI2CPort = I2C.Port.kOnboard;
  private I2C.Port navxI2CPort = I2C.Port.kMXP;
  int i2cPortId = 0x70;
  public final int ColorSensorid = 0;
  public ArrayList<Alliance> ballQ = new ArrayList<Alliance>();

  // Senses colors
  private final ColorSensorV3 frontColorSensor = new ColorSensorV3(rioI2CPort);
  private final ColorSensorV3 backColorSensor = new ColorSensorV3(navxI2CPort);
  // Matches colors
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.562, 0.351, 0.100);
  public Color frontDetectedColor;
  public Color backDetectedColor;

  // Alliance Stuff
  public Alliance curAlliance;
  public Alliance oppAlliance;

  // Balls
  private Alliance frBallType;
  private Alliance prevFr;

  public Alliance bAlliance;
  public Alliance fAlliance;



  /** Creates a new ColorSensorSubsystem. */
  public SUB_ColorSensor() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);

    curAlliance = DriverStation.getAlliance();

    if(curAlliance==Alliance.Red){oppAlliance=Alliance.Blue;}
    else{oppAlliance=Alliance.Red;}

  }
  
  /**
   * Whether the limelight has any valid targets
   * Updates the ball queue periodically
   */
  @Override
  public void periodic() {

    curAlliance = DriverStation.getAlliance();

    if(curAlliance==Alliance.Red){oppAlliance=Alliance.Blue;}
    else{oppAlliance=Alliance.Red;}

    fAlliance = readSensor(Constants.FRONT_COLOR_SENSOR_ID);
    bAlliance = readSensor(Constants.BACK_COLOR_SENSOR_ID);
    
    if (!isUnknown(fAlliance) && fAlliance!=prevFr) {
      pushQ(fAlliance);
    } 
    
    prevFr = fAlliance;
  }



  /**
   * Grabs the color detected from the color sensor at the current I2C port
   * @return a Alliance color, either Red, Blue, or Invalid.
   */
  public Alliance colorToAlliance(int id) {
    final double idealBlueConfidence = 0.85;
    final double idealRedConfidence = 0.92;
    Alliance colorString;
    ColorMatchResult match;
    
    switch(id){
      case 0: match = colorMatcher.matchClosestColor(frontDetectedColor); break;
      case 1: match = colorMatcher.matchClosestColor(backDetectedColor); break;
      default: match = colorMatcher.matchClosestColor(frontDetectedColor); break;
    }

    if (match.color == kBlueTarget) {
      colorString = Alliance.Blue;
    } else if (match.color == kRedTarget) {
      colorString = Alliance.Red;
    } else {
      colorString = Alliance.Invalid;
    }

    if (colorString==Alliance.Blue && match.confidence <= idealBlueConfidence) {
     colorString = Alliance.Invalid;
    }

    if (colorString==Alliance.Red && match.confidence <= idealRedConfidence) {
      colorString = Alliance.Invalid;
     }

    return colorString;
  }
  /**
   * Transcribes an alliance to a string
   * @param all An alliance
   * @return A string representing the alliance
   */
  public String allianceToColor(Alliance all){
    switch (all){
      case Red:
        return "Red";
      case Blue:
        return "Blue";
      default:
        return ("Unknown");
    }
  }
  /**
   * Reads the sensor for a color value
   * @param newId the id of the color sensor you want to read from
   * @return a string with the color of the value read from the sensor
   */
  public Alliance readSensor(int newId){
    
    switch (newId){
      case 0:
        frontDetectedColor = frontColorSensor.getColor();
        break;
      case 1:
        backDetectedColor = backColorSensor.getColor();
        break;
    }
    return colorToAlliance(newId);
  }

  /**
  * Given an alliance it will return whether the alliance is our alliance
  * @param ball Alliance Given
  * @return Boolean value of whether the alliance is our alliance
  */
  public boolean isAlliance(Alliance ball){
    return ball==curAlliance;
  }
  /**
  * Given an alliance it will return whether the alliance is our alliance
  * @param ball Alliance Given
  * @return Boolean value of whether the alliance is our alliance
  */
  public boolean isAlliance(Supplier<Alliance> ball){
    return ball.get()==curAlliance;
  }
  /**
  * Given an alliance it will return whether the alliance is the opposite alliance
  * @param ball Alliance Given
  * @return Boolean value of whether the alliance is the opposite alliance
  */
  public boolean isOpp(Alliance ball){
    return ball==oppAlliance;
  }
  /**
  * Given an alliance it will return whether the alliance is the opposite alliance
  * @param ball Alliance Given
  * @return Boolean value of whether the alliance is the opposite alliance
  */
  public boolean isOpp(Supplier<Alliance> ball){
    return ball.get()==oppAlliance;
  }
  /**
  * Given an alliance it will return whether the alliance is unknown
  * @param ball Alliance Given
  * @return Boolean value of whether the alliance is unknown
  */
  public boolean isUnknown(Alliance ball){
    return ball==Alliance.Invalid;
  }
  /**
  * Enqueues a ball onto the ball queue
  * @param ball The ball to be enqueued
  */
  public void pushQ(Alliance ball){
    ballQ.add(ball);
  }
  /**
  * Dequeues the first ball if the queue is not empty.
  */
  public void popQ(){
    if (ballQ.size()!=0){
      ballQ.remove(0);
    }
  }
  /**
  * Clears the ball queue.
  */
  public void eraseQ(){
    ballQ.clear();
  }
  /**
  * Returns the front-most ball, if the queue is empty, returns unknown.
  * @return Front-most ball or Alliance.Invalid.
  */
  public Alliance peekQ(){
    if (ballQ.size()==0){
      return Alliance.Invalid;
    }
    return ballQ.get(0);
  }



}
