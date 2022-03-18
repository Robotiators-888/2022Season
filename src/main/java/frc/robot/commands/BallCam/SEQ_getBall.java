package frc.robot.commands.BallCam;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CMD_setDrive;
import frc.robot.commands.CanalZeroToOneBottom;
import frc.robot.commands.IndexBottomToTop;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SUB_CameraData;

public class SEQ_getBall extends SequentialCommandGroup {

    public SEQ_getBall(SUB_CameraData cam, Drivetrain drive, CanalSubsystem canal, IntakeSubsystem intake,
            IndexSubsystem index, boolean camSel) {
        if (camSel) {
            addCommands(
                    new CMD_alignToBall(cam, drive, camSel).withTimeout(5),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_DriveToBall(cam, drive, camSel).withTimeout(5),
                            sequence(
                                    new CanalZeroToOneBottom(canal, index),
                                    new IndexBottomToTop(canal, index),
                                    new IntakeSpin(intake, 0.75))),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_setDrive(drive, -0.60, -0.60).withTimeout(2),
                            sequence(
                                    new CanalZeroToOneBottom(canal, index),
                                    new IndexBottomToTop(canal, index),
                                    new IntakeSpin(intake, 0.75))),
                    new InstantCommand(() -> intake.pistonSet(false), intake)

            );
        } else {
            addCommands(
                    new CMD_alignToBall(cam, drive, camSel).withTimeout(5),
                    new InstantCommand(() -> intake.pistonSet(false), intake),
                    deadline(
                            new CMD_DriveToBall(cam, drive, camSel).withTimeout(5),
                            sequence(
                                    new CanalZeroToOneBottom(canal, index),
                                    new IndexBottomToTop(canal, index))),
                    deadline(
                            new CMD_setDrive(drive, 0.60, 0.60).withTimeout(2),
                            sequence(
                                    new CanalZeroToOneBottom(canal, index),
                                    new IndexBottomToTop(canal, index))),
                    new InstantCommand(() -> intake.pistonSet(false), intake)

            );
        }

    }
}