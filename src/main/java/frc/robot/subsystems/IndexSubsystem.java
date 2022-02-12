// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.ColorMatch;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import frc.robot.Constants;


public class IndexSubsystem extends SubsystemBase {

  // Should this be in constants?
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
  private Color detectedColor;




  // Motors
  private TalonSRX front = new TalonSRX(Constants.FRONT_INDEX_ID);
  private TalonSRX tower = new TalonSRX(Constants.TOWER_INDEX_ID);
  private TalonSRX back = new TalonSRX(Constants.BACK_INDEX_ID);
  
  public static void MuxChangeI2cPort(I2C.Port I2CPort,int deviceAdress,int newPort) {
  I2C I2CObject = new I2C(I2CPort,deviceAdress);
  int i2cPortId = 0x70; // MUX I2C address
  // and you simply write a single byte with the desired multiplexed output
  // number to that port
   boolean failed = I2CObject.write(i2cPortId, newPort);
   if (failed) {
      System.out.println("Failed to write to MUX over I2C");
    }
    I2CObject.close();
  }

  /** Creates a new ColorSensorSubsystem. */
  public IndexSubsystem() {

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kBlackTarget);
    colorMatcher.addColorMatch(kRedTarget);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // NOTE: change ColorSensorid to change which color sensor is used
   //MuxChangeI2cPort(i2cPort,i2cPortId,ColorSensorid);

  }

  public boolean getPosition(int id){
    int newId = 0;
    if (id==1){
      // placeholder
      newId = 0;
    } else if (id==2){
      // placeholder
      newId = 1;
    }
    MuxChangeI2cPort(i2cPort,i2cPortId,newId);
    detectedColor = colorSensor.getColor();


    return !(findColor().equals("Unknown"));
  }

  public String getColor(int id){
    int newId = 0;
    if (id==1){
      // placeholder
      newId = 0;
    } else if (id==2){
      // placeholder
      newId = 1;
    }
    MuxChangeI2cPort(i2cPort,i2cPortId,newId);
    detectedColor = colorSensor.getColor();


    return findColor();
  }



  public double[] findRGB() {
    double[] RGBArray = new double[3];
    RGBArray[0] = detectedColor.red;
    RGBArray[1] = detectedColor.blue;
    RGBArray[2] = detectedColor.green;

    return RGBArray;
  }

  public String findColor() {
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

  public void feed(){
    setSpeedTower(Constants.TOWER_BELT_SPEED);
  }
  
  public void stopFeed(){
    setSpeedTower(0);
  }

  public void setSpeedFront(double speed){
     // Front belt, two sets of belts.
     front.set(TalonSRXControlMode.PercentOutput, speed);
  }
  
  public void setSpeedBack(double speed){
    // Back belt, two sets of belts.
    back.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setSpeedTower(double speed){
    // Top-most belt, move to get it into shooter
    tower.set(TalonSRXControlMode.PercentOutput,speed);
  }

}