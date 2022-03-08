// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.Constants;

public class SUB_LED extends SubsystemBase {

  private AddressableLED LED = new AddressableLED(Constants.LED_PORT);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
  
  
  /** Creates a new LEDSubsystem. */
  public SUB_LED() {
    LED.setLength(Constants.LED_LENGTH);
    LED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LED.setData(buffer);
  }

  public void setLED(int id, int h, int s, int v ){
    buffer.setHSV(id, h, s, v);
  }

  public void setRGBLED(int id, int r, int g, int b ){
    buffer.setRGB(id, r, g, b);
  }
}
