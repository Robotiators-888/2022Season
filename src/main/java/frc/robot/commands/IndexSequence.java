// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.BallIntake;
import frc.robot.commands.PistonOutCmd;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexSequence extends SequentialCommandGroup {
  IndexSubsystem index;
  IntakeSubsystem intake;
  PistonOutCmd pistonOutCmd;
  BallIntake ballIntake;
  /** Creates a new IndexSequence. */
  public IndexSequence(IntakeSubsystem _intake, IndexSubsystem _index) {
    this.index = _index;
    this.intake = _intake;
    pistonOutCmd = new PistonOutCmd(this.intake);
    ballIntake = new BallIntake(this.index, this.intake);
    // Add your coommands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(this.pistonOutCmd, this.ballIntake);
  }
}
