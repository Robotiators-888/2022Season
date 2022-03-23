// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallIndexing;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Index;
import frc.robot.Constants;

/** The CanalZeroToOneBottom command is the escape sequence from the zero ball state. 
 * TThe command runs the canal until a ball is detected in the bottom banner.
 * This command assumes that there are zero balls within the robot.
 */
public class CMD_CanalZeroToOneBottom extends CommandBase {

  private SUB_Canal canal;
  private SUB_Index index;
  private boolean isDone = false;


  

  /** Creates a new CanalZeroToOneBottom. */
  public CMD_CanalZeroToOneBottom(SUB_Canal canalArgs, SUB_Index indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
    addRequirements(canal,index);

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