// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;

public class CMD_TeleopClimber extends CommandBase {
  private SUB_Climber climber;
  private double triggerSpeed;
  /** Creates a new teleopClimber. */
  public CMD_TeleopClimber(SUB_Climber climb, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climb;
    this.triggerSpeed = speed;

    addRequirements(climber);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    climber.speedSet(triggerSpeed);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.speedSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
