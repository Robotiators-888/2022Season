package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Shooter;

public class SEQ_dumbShot extends SequentialCommandGroup {

    public SEQ_dumbShot(Shooter shootIn, Drivetrain driveIn, IndexSubsystem indexIn, int RPM) {
        addCommands(
                race(
                        new ShooterRPM(shootIn, 2000),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new indexRun(indexIn, 0.75).withTimeout(2)))

        );

    }
}