// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_antiTip extends CommandBase {
  /** Creates a new CMD_antiTip. */
  private double motorSpeed;
  private SUB_Drivetrain drive;
  private AHRS navx = new AHRS();

  public CMD_antiTip(SUB_Drivetrain drivetrain, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    motorSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setMotors(motorSpeed, -1 * motorSpeed);
    System.out.println("antiTip motorSpeed: "+motorSpeed);
    if (Math.abs(navx.getRoll()) < 0.1) {
      end(true);
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
