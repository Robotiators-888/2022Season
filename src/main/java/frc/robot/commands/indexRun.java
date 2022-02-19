// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class indexRun extends CommandBase {

  private IndexSubsystem index;
  private boolean isDone = false;
  private double speed;
  
  /** Creates a new teleopIndex. */
  public indexRun(IndexSubsystem indexArgs, double speedArgs) {
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
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleopIndex.java
    if (!(index.getBallPosition(1))){
      isDone = false;
      index.setSpeedTower(Constants.BELT_SPEED);
    } else {
      isDone = true;
    }
=======
    
    index.setSpeedTower(speed);
>>>>>>> d9ea0b1 (Added manual index in and out. Added organization of the index):src/main/java/frc/robot/commands/indexRun.java

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
