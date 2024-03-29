// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShooterSpin extends CommandBase {
  /** Creates a new ShooterSpin. */
  private SUB_Shooter shoot;
  private double shootSpeed;

  public CMD_ShooterSpin(SUB_Shooter subsystem, double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = subsystem;

    this.shootSpeed = shooterSpeed;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shoot.setSpeed(shootSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
