// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Canal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Canal;

/** The command canalRun runs the canal belts either both in or both out */
public class CMD_canalRun extends CommandBase {

  private SUB_Canal canal;
  private double speed;
  /** Creates a new canalRun. */
  public CMD_canalRun(SUB_Canal canalArgs, double speedArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.speed = speedArgs;
    addRequirements(canal);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canal.setSpeedBack(speed);
    canal.setSpeedFront(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //canal.setSpeedFront(0);
    //canal.setSpeedBack(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}