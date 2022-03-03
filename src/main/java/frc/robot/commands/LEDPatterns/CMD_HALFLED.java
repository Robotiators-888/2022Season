// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDPatterns;

import frc.robot.subsystems.SUB_LED;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_HALFLED extends CommandBase {

  private int counterID = 0;
  private SUB_LED LED;

  /** Creates a new CMD_CHASELED. */
  public CMD_HALFLED(SUB_LED LEDArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED = LEDArgs;
    addRequirements(LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (counterID>=Constants.LED_LENGTH){
      counterID = 0;
    }

    if (counterID<Constants.LED_LENGTH/2){
      LED.setRGBLED(counterID,255,8,0);
    } else {
      LED.setLED(counterID,0,47,167);
    }

    counterID++;
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
