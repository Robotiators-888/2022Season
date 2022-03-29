// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climber.*;
import frc.robot.subsystems.SUB_Climber;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SEQ_AutoClimber extends SequentialCommandGroup {

  /** Creates a new SEQ_AutoClimber. */
  public SEQ_AutoClimber(SUB_Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //new WaitUntilCommand(()->(climber.getPitch()<=-35))
    //   new WaitUntilCommand(() -> (climber.getMin() > -35)),
    //32
    addCommands(

      // Low
      new CMD_ClimberHome(climber),

      // High
      new CMD_ClimberToPosition(climber,175),
      new CMD_ClimberToPosition(climber,120),

      // Wait until stable
      new WaitUntilCommand(() -> (climber.getMin() > -25 && Math.abs(climber.getYaw()) < 7)), //wait until the min  angle is greater than -30
      new WaitCommand(0.5),
      new CMD_ClimberHome(climber),
      new WaitCommand(0.5),

      // Traversal
      new CMD_ClimberToPosition(climber,175,28,35),
      new CMD_ClimberToPosition(climber,120,28,35)
    );
  }
}
