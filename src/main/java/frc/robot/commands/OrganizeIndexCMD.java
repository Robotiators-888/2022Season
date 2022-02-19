// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class OrganizeIndexCMD extends CommandBase {
  private IndexSubsystem index;
  private boolean isDone = false;
  /** Creates a new OrganizeIndexCMD. */
  public OrganizeIndexCMD(IndexSubsystem indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    if ( !(index.getPosition(1)) && index.getPosition(2) ){
<<<<<<< HEAD
      isDone = false;
=======
>>>>>>> 63b7852 (Fixed previous index organization and index stop)
      index.setSpeedTower(0.75);
    } else {
      isDone = true;
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
