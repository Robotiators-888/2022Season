package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Shooter;

public class SEQ_dumbShot extends SequentialCommandGroup {

    public SEQ_dumbShot(Shooter shootIn, IndexSubsystem indexIn, int RPM) {
        addCommands(
                race(
                        new ShooterRPM(shootIn, RPM),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> ((shootIn.getRPM() <= RPM + 500) && (shootIn.getRPM() >= RPM - 500))),
                                new indexRun(indexIn, 0.75).withTimeout(2)))

        );

    }
}