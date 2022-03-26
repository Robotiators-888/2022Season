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
    addCommands(
      new CMD_ClimberHome(climber),
      new CMD_ClimberToPosition(climber,175),
      
      new CMD_ClimberToPosition(climber,120),
      //new WaitCommand(5),
      new WaitUntilCommand(() -> (climber.getMin() > -30)), //wait until the min  angle is greater than -35
      new CMD_ClimberHome(climber),
      new WaitCommand(1),
      new CMD_ClimberToPosition(climber,175),
      new CMD_ClimberToPosition(climber,120)
    );
  }
}
