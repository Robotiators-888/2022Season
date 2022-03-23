// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CanalSubsystem;


public class CMD_AutoIntake extends CommandBase {
  /** Creates a new CMD_AutoIntake. */
  private IntakeSubsystem intake;
  private CanalSubsystem canal;

  public CMD_AutoIntake(IntakeSubsystem intakeArgs, CanalSubsystem canalArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeArgs;
    this.canal = canalArgs;

    addRequirements(intake,canal);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeSpeedSet(0.5);
    canal.setSpeedFront(-0.75);
    canal.setSpeedBack(-0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeSpeedSet(0);
    canal.setSpeedFront(0);
    canal.setSpeedBack(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
