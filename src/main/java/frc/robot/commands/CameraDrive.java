// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.UDP.BallDataPacket;

public class CameraDrive extends CommandBase {
  /** Creates a new CameraDrive. */
  Drivetrain drive;
  double cameraXValue;
  public CameraDrive(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(BallDataPacket.getX() > 10){
      cameraXValue = 10;
    }else{
      cameraXValue = BallDataPacket.getX();
    }
    drive.setMotors((0.05)*(BallDataPacket.getX()),(-0.05)*(BallDataPacket.getX()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
