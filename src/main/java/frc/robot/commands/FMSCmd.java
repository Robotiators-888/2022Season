// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FMSSubsystem;

public class FMSCmd extends CommandBase {
  /** Creates a new FMSCmd. */
  Alliance alliance;
  MatchType matchtype;
  String EventName;
  String GameSpecMsg;
  int matchNumber;
  int allianceLocation;
  double matchTime;


  public FMSCmd(Subsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_fmssubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_fmssubsystem.findInfo(alliance, matchtype, EventName, GameSpecMsg, matchNumber, allianceLocation, matchTime);
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
