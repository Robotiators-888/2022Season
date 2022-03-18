package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexRun;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SEQ_limeShot extends SequentialCommandGroup {

    /**
     * command to automatically turn to goal and shoot using the limelight
     * @param shootIn
     * @param driveIn
     * @param indexIn
     * @param limeIn
     * @param aimHigh
     */
    public SEQ_limeShot(Shooter shootIn, Drivetrain driveIn, IndexSubsystem indexIn, Limelight limeIn, boolean aimHigh) {
        addCommands(
                new InstantCommand(() -> limeIn.setLed(3), limeIn),
                new CMD_limeAlign(limeIn, driveIn),
                race(
                        new CMD_limeSpin(limeIn, shootIn, aimHigh),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new indexRun(indexIn, 0.75).withTimeout(1)))
        );
    }
}