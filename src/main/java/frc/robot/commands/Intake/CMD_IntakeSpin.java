// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.subsystems.SUB_Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_IntakeSpin extends CommandBase {
  SUB_Intake Intake;
  double speed;

  /** Creates a new IntakeMotorTest. */
  public CMD_IntakeSpin(SUB_Intake subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = subsystem;
    this.speed = speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.intakeSpeedSet(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.intakeSpeedSet(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
