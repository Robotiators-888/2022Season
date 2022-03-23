// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_LED;
import frc.robot.Constants;

public class CMD_RAINBOWLED extends CommandBase {

  private int counterID = 0;
  private SUB_LED LED;

  private int m_rainbowFirstPixelHue = 0;
  final int hue = 0;

  /** Creates a new LEDCommand. */
  public CMD_RAINBOWLED(SUB_LED LEDArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED = LEDArgs;
    addRequirements(LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counterID>=60){
      counterID=0;
    }
    final var hue = (m_rainbowFirstPixelHue + (counterID * 180 / Constants.LED_LENGTH)) % 180;
    LED.setLED(counterID,hue,255,128);
    counterID++;
    m_rainbowFirstPixelHue+=1;
    m_rainbowFirstPixelHue %= 180;
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
