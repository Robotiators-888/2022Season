// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.Constants; 
public class IndexBottomToTop extends CommandBase {

  private CanalSubsystem canal;
  private IndexSubsystem index;
  private boolean isDone = false;


  /** Creates a new MegaCommand. */
  public IndexBottomToTop(CanalSubsystem canalArgs, IndexSubsystem indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    addRequirements(index,canal);

    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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