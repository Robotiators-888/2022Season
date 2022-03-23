package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Index.CMD_indexRun;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Shooter;

public class SEQ_limeShot extends SequentialCommandGroup {

    /**
     * command to automatically turn to goal and shoot using the limelight
     * @param shootIn
     * @param driveIn
     * @param indexIn
     * @param limeIn
     * @param aimHigh
     */
    public SEQ_limeShot(SUB_Shooter shootIn, SUB_Drivetrain driveIn, SUB_Index indexIn, SUB_Limelight limeIn, boolean aimHigh) {
        addCommands(
                new InstantCommand(() -> limeIn.setLed(3), limeIn),
                new CMD_limeAlign(limeIn, driveIn),
                race(
                        new CMD_limeSpin(limeIn, shootIn, aimHigh),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new CMD_indexRun(indexIn, 0.75).withTimeout(1)))
        );
    }
}