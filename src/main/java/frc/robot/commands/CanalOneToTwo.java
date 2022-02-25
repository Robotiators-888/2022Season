// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.Constants;
public class CanalOneToTwo extends CommandBase {

  private CanalSubsystem canal;
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

    if (index.readBottomBanner()) {

      if (index.readTopBanner()) {
        States activeState = States.TWO_BALL;
        return activeState;
      } 

      States activeState = States.ONE_BALL_BOTTOM;
      return activeState;
    } else {
      States activeState = States.ZERO_BALL;
      return activeState;
    }
  }

  /** Creates a new MegaCommand. */
  public CanalOneToTwo(CanalSubsystem canalArgs, IndexSubsystem indexArgs) {
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
        break;
      case ONE_BALL_BOTTOM:
        break;
      case ONE_BALL_TOP:
        canal.setSpeedBack(Constants.BELT_SPEED);
        canal.setSpeedFront(Constants.BELT_SPEED);
        break;
      case TWO_BALL:
        break;

    }
    isDone = true;
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