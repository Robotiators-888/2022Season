// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexBottomToTopBanner extends CommandBase {

  private IndexSubsystem index;
  private boolean isDone = false;
  private double speed;
  
  /** Creates a new teleopIndex. */
  public IndexBottomToTopBanner(IndexSubsystem indexArgs, double speedArgs) {
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
    if (!(index.readTopBanner())){
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
