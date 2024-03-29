// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallIndexing;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Index;
import frc.robot.Constants; 
/** The command IndexBottomToTop runs the index if there is a ball in the bottom and not a ball in the top */
public class CMD_IndexBottomToTop extends CommandBase {

  private SUB_Canal canal;
  private SUB_Index index;
  private boolean isDone = false;


  /** Creates a new IndexBottomToTOp. */
  public CMD_IndexBottomToTop(SUB_Canal canalArgs, SUB_Index indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    addRequirements(index);

    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If there isn't a ball at the top and there is a ball at the bottom
    // run the index until there is a ball at the top
    if (!index.readTopBanner() && index.readBottomBanner()){
      isDone = false;
      index.setSpeedTower(Constants.BELT_SPEED);
      canal.setSpeedBack(-1 * Constants.BELT_SPEED);
      canal.setSpeedFront(-1 * Constants.BELT_SPEED);
    } else {
        isDone = true;
      }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setSpeedTower(0);
      canal.setSpeedBack(0);
      canal.setSpeedFront(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}