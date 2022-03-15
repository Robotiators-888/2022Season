package frc.robot.commands.BallCam;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Shooter;

public class SEQ_getBall extends SequentialCommandGroup {

    public SEQ_getBall(Shooter shootIn, IndexSubsystem indexIn, int RPM) {
        addCommands(


        );

    }
}