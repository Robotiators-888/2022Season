// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class CMD_AutoIntake extends CommandBase {
  /** Creates a new CMD_AutoIntake. */
  IntakeSubsystem intake;
  double speed;

  public CMD_AutoIntake(IntakeSubsystem subsystem, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = subsystem;
    this.speed = intakeSpeed;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getPosition()) {
      intake.intakeSpeedSet(speed);
    } else {
      intake.intakeSpeedSet(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
