package frc.robot.commands.BallCam;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BallIndexing.CMD_CanalZeroToOneBottom;
import frc.robot.commands.BallIndexing.CMD_IndexBottomToTop;
import frc.robot.commands.Drivetrain.CMD_setDrive;
import frc.robot.commands.Intake.CMD_IntakeSpin;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_CameraData;

public class SEQ_getBall extends SequentialCommandGroup {

    public SEQ_getBall(SUB_CameraData cam, SUB_Drivetrain drive, SUB_Canal canal, SUB_Intake intake,
            SUB_Index index, boolean camSel) {
        if (camSel) {
            addCommands(
                    new CMD_alignToBall(cam, drive, camSel).withTimeout(5),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_DriveToBall(cam, drive, camSel).withTimeout(5),
                            sequence(
                                    new CMD_CanalZeroToOneBottom(canal, index),
                                    new CMD_IndexBottomToTop(canal, index),
                                    new CMD_IntakeSpin(intake, 0.75))),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_setDrive(drive, -0.65, -0.65).withTimeout(2),
                            sequence(
                                    new CMD_CanalZeroToOneBottom(canal, index),
                                    new CMD_IndexBottomToTop(canal, index),
                                    new CMD_IntakeSpin(intake, 0.75))),
                    new InstantCommand(() -> intake.pistonSet(false), intake)

            );
        } else {
            addCommands(
                    new CMD_alignToBall(cam, drive, camSel).withTimeout(5),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_DriveToBall(cam, drive, camSel).withTimeout(5),
                            sequence(
                                    new CMD_CanalZeroToOneBottom(canal, index),
                                    new CMD_IndexBottomToTop(canal, index))),
                    deadline(
                            new CMD_setDrive(drive, 0.65, 0.65).withTimeout(2),
                            sequence(
                                    new CMD_CanalZeroToOneBottom(canal, index),
                                    new CMD_IndexBottomToTop(canal, index))),
                    new InstantCommand(() -> intake.pistonSet(false), intake)

            );
        }

    }
}