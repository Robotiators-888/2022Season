// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Canal;
import frc.robot.Constants;

public class CMD_CanalZeroToOneBottom extends CommandBase {

  private SUB_Canal canal;
  private boolean isDone = false;


  

  /** Creates a new MegaCommand. */
  public CMD_CanalZeroToOneBottom(SUB_Canal canalArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    addRequirements(canalArgs, indexArgs);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!index.readBottomBanner()){
      isDone = false;
      canal.setSpeedBack(-Constants.BELT_SPEED);
      canal.setSpeedFront(-Constants.BELT_SPEED);
    } else{
      isDone = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canal.setSpeedBack(0);
    canal.setSpeedFront(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}