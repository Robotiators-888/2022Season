// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.IndexSubsystem;

public class indexCanalOut extends CommandBase {
  private CanalSubsystem canal;
  private IndexSubsystem index;
  
  /** Creates a new canalOut. */
  public indexCanalOut(CanalSubsystem canalArgs, IndexSubsystem indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    
    addRequirements(canal);
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    index.setSpeedTower(-Constants.BELT_SPEED);
    canal.setSpeedBack(Constants.BELT_SPEED);
    canal.setSpeedFront(Constants.BELT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canal.setSpeedBack(0);
    canal.setSpeedFront(0);
    index.setSpeedTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
