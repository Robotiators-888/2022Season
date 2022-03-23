package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Index.CMD_indexRun;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Shooter;

public class SEQ_dumbShot extends SequentialCommandGroup {

    public SEQ_dumbShot(SUB_Shooter shootIn, SUB_Index indexIn, int RPM) {
        addCommands(
                race(
                        new CMD_ShooterRPM(shootIn, RPM),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> ((shootIn.getRPM() <= (-RPM + 200)) && (shootIn.getRPM() >= (-RPM - 200)))),
                                new CMD_indexRun(indexIn, 0.75).withTimeout(1.25)))

        );

    }
}