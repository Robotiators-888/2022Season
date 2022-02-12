// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IndexTest extends CommandBase {

  private IndexSubsystem subsystem;
  /** Creates a new IndexTest. */
  public IndexTest(IndexSubsystem m_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = m_subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean pos = subsystem.getPosition(0);
    String color = subsystem.getColor(0);
    SmartDashboard.putBoolean("Val at pos 0: ", pos);
    SmartDashboard.putString("Color: ",color);
    subsystem.feed();
    subsystem.stopFeed();
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
