// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallIndexing;

import frc.robot.subsystems.SUB_Index;


import edu.wpi.first.wpilibj2.command.CommandBase;

/** The command IndexBottomToTopBanner is an escape sequence for the one ball bottom state
 * The command runs the index until there is a ball at the top
 * This command assumes there is a ball at the bottom banner
 */
public class CMD_IndexBottomToTopBanner extends CommandBase {

  private SUB_Index index;
  private boolean isDone = false;
  private double speed;
  
  /** Creates a new IndexBottomToTopBanner. */
  public CMD_IndexBottomToTopBanner(SUB_Index indexArgs, double speedArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speedArgs;
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
    if ((index.readTopBanner())){
      isDone = true;
    } else {
      isDone = false;
      index.setSpeedTower(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setSpeedTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
