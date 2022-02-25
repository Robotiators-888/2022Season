// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.Constants; 
public class CanalToUpCMD extends CommandBase {

  private CanalSubsystem canal = new CanalSubsystem();
  private IndexSubsystem index;
  private boolean isDone = false;

  // create States
  public enum States {
    ONE_BALL_TOP,
    ONE_BALL_BOTTOM,
    TWO_BALL,
    ZERO_BALL,
  }

  public States initalizeStates() {
    if (index.readTopBanner()) {

      if (index.readBottomBanner()) {
        States activeState = States.TWO_BALL;
        return activeState;
      }
      
      States activeState = States.ONE_BALL_TOP;
      return activeState;
    } else {
      States activeState = States.ZERO_BALL;
      return activeState;
    }
  }

  /** Creates a new MegaCommand. */
  public CanalToUpCMD(CanalSubsystem canalArgs, IndexSubsystem indexArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.index = indexArgs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isDone = false;
    switch (initalizeStates()) {
      case ZERO_BALL:
        canal.setSpeedBack(-Constants.BELT_SPEED);
        canal.setSpeedFront(-Constants.BELT_SPEED);
        index.setSpeedTower(0);
        break;
      case ONE_BALL_BOTTOM:
        index.setSpeedTower(Constants.BELT_SPEED);
        canal.setSpeedBack(Constants.BELT_SPEED);
        canal.setSpeedFront(Constants.BELT_SPEED);
        break;
      case ONE_BALL_TOP:
        canal.setSpeedBack(Constants.BELT_SPEED);
        canal.setSpeedFront(Constants.BELT_SPEED);
        break;
      case TWO_BALL:
        index.setSpeedTower(0);
        canal.setSpeedBack(0);
        canal.setSpeedFront(0);
        break;

    }
    isDone = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
