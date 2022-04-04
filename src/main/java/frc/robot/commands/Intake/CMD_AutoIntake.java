// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Intake;

public class CMD_AutoIntake extends CommandBase {
  SUB_Canal canal;
  SUB_Intake intake;

  /** Creates a new CMD_AutoIntake. */
  public CMD_AutoIntake(SUB_Canal canalArgs, SUB_Intake intakeArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.canal = canalArgs;
    this.intake = intakeArgs;
    
    // Not adding requirements because this command will interfere with auto

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeSpeedSet(0.5);
    canal.setSpeedBack(-0.75);
    canal.setSpeedFront(-0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeSpeedSet(0);
    canal.setSpeedBack(0);
    canal.setSpeedFront(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
