// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Canal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Index;

/** The command indexCanalOut runs both the index and canal in the direction out. */
public class CMD_indexCanalOut extends CommandBase {
  private SUB_Canal canal;
  private SUB_Index index;
  
  /** Creates a new canalOut. */
  public CMD_indexCanalOut(SUB_Canal canalArgs, SUB_Index indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    
    addRequirements(canal, index);

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
