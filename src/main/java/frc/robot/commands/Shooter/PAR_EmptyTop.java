// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter.CMD_ShooterRPM;
import frc.robot.commands.Index.CMD_indexRun;

import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PAR_EmptyTop extends ParallelCommandGroup {
  /** Creates a new PAR_EmptyTop. */
  public PAR_EmptyTop(SUB_Index index, SUB_Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_indexRun(index, 0.75),
      new CMD_ShooterRPM(shooter, 1750)
    );
  }
}
