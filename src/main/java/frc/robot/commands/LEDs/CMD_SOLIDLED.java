// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.subsystems.SUB_LED;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;;

public class CMD_SOLIDLED extends CommandBase {

  private int counterID = 0;
  private SUB_LED LED;
  private Alliance color;
  private int[] rgbVals = new int[3];
  private boolean invalid = false;

  /** Creates a new CMD_CHASELED. */
  public CMD_SOLIDLED(SUB_LED LEDArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED = LEDArgs;
    addRequirements(LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    color = DriverStation.getAlliance();

    if (color == Alliance.Red){
      rgbVals[0] = 255;
      rgbVals[1] = 0;
      rgbVals[2] = 0;
    } else if (color == Alliance.Blue){
      rgbVals[0] = 0;
      rgbVals[1] = 47;
      rgbVals[2] = 167;
    } else{
      invalid = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!invalid){
      if (counterID>=Constants.LED_LENGTH){
        counterID = 0;
      }

      LED.setRGBLED(counterID,rgbVals[0],rgbVals[1],rgbVals[2]);

      counterID++;
  } else {
    
    if (counterID>=Constants.LED_LENGTH){
      counterID = 0;
    }

    if (counterID<Constants.LED_LENGTH/2){
      LED.setRGBLED(counterID,255,8,0);
    } else {
      LED.setRGBLED(counterID,0,47,167);
    }

    counterID++;

    }
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
