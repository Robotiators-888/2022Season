// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeMotorTest;
import frc.robot.commands.teleopIndex;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BallIntake extends ParallelCommandGroup {
  IndexSubsystem index;
  IntakeSubsystem intake;
  IntakeMotorTest intakeCmd;
  teleopIndex indexCmd;

  /** Creates a new ParallelCmd. */
  public BallIntake(IndexSubsystem index, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeCmd = new IntakeMotorTest(intake);
    this.indexCmd = new teleopIndex(index);
   
    addCommands(this.intakeCmd,this.indexCmd);
  }

}
