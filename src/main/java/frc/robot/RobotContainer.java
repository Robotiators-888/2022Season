// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.BallCam.SEQ_getBall;
import frc.robot.commands.BallIndexing.CMD_CanalZeroToOneBottom;
import frc.robot.commands.BallIndexing.CMD_IndexBottomToTop;
import frc.robot.commands.BallIndexing.CMD_IndexBottomToTopBanner;
import frc.robot.commands.Climber.CMD_ClimberSpeed;
import frc.robot.commands.ColorSensor.CMD_ColorOnDashboard;
import frc.robot.commands.ColorSensor.CMD_ManageBallQueue;
import frc.robot.commands.Drivetrain.CMD_teleopDrive;
import frc.robot.commands.Index.CMD_indexRun;
import frc.robot.commands.Intake.CMD_IntakeSpin;
import frc.robot.commands.LEDs.*;
import frc.robot.commands.LimeLight.*;
import frc.robot.commands.Shooter.CMD_ShooterManualRPM;
import frc.robot.commands.Shooter.CMD_changeSetpoint;
import frc.robot.commands.Shooter.SEQ_dumbShot;
import frc.robot.commands.Shooter.CMD_ShooterRPM;
import frc.robot.commands.Shooter.CMD_ShooterSpin;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Drivetrain;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Canal.*;
import frc.robot.commands.Canal.CSRejection.CMD_AcceptAllianceBall;
import frc.robot.commands.Canal.CSRejection.CMD_CanalRejectBall;
import frc.robot.commands.Canal.CSRejection.CMD_RescindAllianceBall;
import frc.robot.subsystems.SUB_LED;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems and commands are defined here...
        private final Field2d field2d = new Field2d();

        // subsystems
        private SUB_Shooter shooter = new SUB_Shooter();
        private SUB_Drivetrain drivetrain = new SUB_Drivetrain(field2d);
        private SUB_Intake intake = new SUB_Intake();
        private SUB_Index index = new SUB_Index();
        private Autonomous autoHelper = new Autonomous(drivetrain);
        private SUB_Canal canal = new SUB_Canal();
        private SUB_Climber climber = new SUB_Climber();
        private SUB_Limelight limelight = new SUB_Limelight();
        private SUB_CameraData cameraData = new SUB_CameraData();
        private SUB_LED LED = new SUB_LED();
        private SUB_ColorSensor colorSensor = new SUB_ColorSensor();
        

        // Controller
        private Joystick controller = new Joystick(Constants.JOYSTICK_PORT);

        JoystickButton C_aButton = new JoystickButton(controller, 1);
        JoystickButton C_bButton = new JoystickButton(controller, 2);
        JoystickButton C_xButton = new JoystickButton(controller, 3);
        JoystickButton C_yButton = new JoystickButton(controller, 4);
        POVButton C_dPadUp = new POVButton(controller, 0);
        POVButton C_dPadDown = new POVButton(controller, 180);
        POVButton C_dPadLeft = new POVButton(controller, 270);
        POVButton C_dPadRight = new POVButton(controller, 90);
        Trigger C_leftTrigger;
        Trigger C_rightTrigger;

        Trigger intakeDown = new Trigger(()->intake.getPosition());
        Trigger bottomIndexTrigger = new Trigger(()->index.readBottomBanner());
        Trigger topIndexTrigger = new Trigger(()->!index.readTopBanner());

        Trigger notRejectingTrigger = new Trigger(()->!canal.rejecting);
        Trigger indexBottomToTopTrigger = topIndexTrigger.and(bottomIndexTrigger).and(notRejectingTrigger);
        
        Trigger rejectBallTrigger = new Trigger(()->colorSensor.isOpp(()->colorSensor.peekQ()));
        Trigger acceptBallTrigger = new Trigger(()->colorSensor.isAlliance(()->colorSensor.peekQ()));
        Trigger rescindBallTrigger = new Trigger(()->colorSensor.isAlliance(colorSensor.readSensor(Constants.BACK_COLOR_SENSOR_ID))).and(notRejectingTrigger);

        // left Joystick
        private Joystick leftJoystick = new Joystick(Constants.LEFTJOYSTICK_PORT);

        JoystickButton L_button2 = new JoystickButton(leftJoystick, 2);
        JoystickButton L_button3 = new JoystickButton(leftJoystick, 3);
        JoystickButton L_button4 = new JoystickButton(leftJoystick, 4);
        JoystickButton L_button5 = new JoystickButton(leftJoystick, 5);
        JoystickButton L_button7 = new JoystickButton(leftJoystick, 7);
        JoystickButton L_button10 = new JoystickButton(leftJoystick, 10);
        JoystickButton L_button11 = new JoystickButton(leftJoystick, 11);
        JoystickButton L_Trigger = new JoystickButton(leftJoystick, 1);

        // right Joytick
        private Joystick rightJoystick = new Joystick(Constants.RIGHTSTICK_PORT);

        JoystickButton R_button5 = new JoystickButton(rightJoystick, 3);
        JoystickButton R_button6 = new JoystickButton(rightJoystick, 4);
        JoystickButton R_button3 = new JoystickButton(rightJoystick, 5);
        JoystickButton R_button4 = new JoystickButton(rightJoystick, 6);
        JoystickButton R_trigger = new JoystickButton(rightJoystick, 1);

        // Auto objects
        SendableChooser<Command> AutoChooser = new SendableChooser<>();
        SendableChooser<Integer> DelayChooser = new SendableChooser<>();

        // ====================================================================
        // Trajectories
        // ====================================================================
        TrajectoryConfig configReversed = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .setReversed(true);

        TrajectoryConfig configForward = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);

        Trajectory straight = autoHelper.getTrajectory("paths/output/straight.wpilib.json");

        Trajectory LS_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p1.wpilib.json");
        Trajectory LS_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p2.wpilib.json");

        Trajectory RS_RB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_RB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_LB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_LB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_threeBall_p1 = autoHelper.getTrajectory("paths/output/RS_threeBall_p1.wpilib.json");
        Trajectory RS_threeBall_p2 = autoHelper.getTrajectory("paths/output/RS_threeBall_p2_v2.wpilib.json");

        Trajectory RS_threeBall_p2_LOW = autoHelper.getTrajectory("paths/output/RS_threeBall_p2_LOW.wpilib.json");

        Trajectory RS_fourBall_p1 = autoHelper.getTrajectory("paths/output/RS_fourBall_p1.wpilib.json");
        Trajectory RS_fourBall_p2 = autoHelper.getTrajectory("paths/output/RS_fourBall_p2.wpilib.json");
        Trajectory RS_fourBall_p3 = autoHelper.getTrajectory("paths/output/RS_fourBall_p3.wpilib.json");

        Trajectory LS_Billiards_p1 = autoHelper.getTrajectory("paths/output/LS_Billiards_p1.wpilib.json");
        Trajectory LS_Billiards_p2 = autoHelper.getTrajectory("paths/output/LS_Billiards_p2.wpilib.json");

        Trajectory LS_Defensive_p1 = autoHelper.getTrajectory("paths/output/LS_twoBall_defensive_p1.wpilib.json");
        Trajectory LS_Defensive_p2 = autoHelper.getTrajectory("paths/output/LS_twoBall_defensive_p2.wpilib.json");

        Trajectory LS_Citrus_p1 = autoHelper.getTrajectory("paths/output/LS_Citrus.wpilib.json");

        Trajectory Str8 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180))),
                        List.of(), new Pose2d(4, 0, new Rotation2d(Units.degreesToRadians(180))), configReversed);

        // ====================================================================
        // Simple autos
        // ====================================================================
        Command limelightHighShot = new SequentialCommandGroup(
                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true));

        Command straightAuto = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        autoHelper.getRamset(Str8));

        Command straightWeaver = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(straight.getInitialPose())),
                        autoHelper.getRamset(straight));

        Command lowDump = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new SEQ_dumbShot(shooter, index, 1800),
                        autoHelper.getRamset(Str8));

        Command lowDumpNoDrive = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new SEQ_dumbShot(shooter, index, 1800));

        // ====================================================================
        // Two Ball command Groups
        // ====================================================================
        Command RS_RB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_RB_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_RB_twoBall_Low_p1),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_RB_twoBall_Low_p2),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_LB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_LB_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_LB_twoBall_Low_p1),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_LB_twoBall_Low_p2),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_twoBall_NC = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_twoBall_Low_p1),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(LS_twoBall_Low_p2),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_twoBall_WC = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_twoBall_Low_p1.getInitialPose())),
                        new InstantCommand(() -> cameraData.setDirection(false), cameraData),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_twoBall_Low_p1).withInterrupt(
                                                        () -> ((cameraData.getY() <= 45) && (cameraData.getY() >= 10)
                                                                        || (Math.abs(cameraData.getX()) > 3)
                                                                                        && (cameraData.getY() <= 30))),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        new InstantCommand(() -> drivetrain.setMotors(0, 0), drivetrain),
                        new SEQ_getBall(cameraData, drivetrain, canal, intake, index, false).withTimeout(6),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        autoHelper.getRamset(LS_twoBall_Low_p2),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        // ====================================================================
        // Three Ball command Groups
        // ====================================================================
        Command RS_threeBall_NC_HIGH = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new SEQ_dumbShot(shooter, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p1),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(5),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new InstantCommand(() -> intake.pistonSet(true), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p2),
                                        new CMD_IntakeSpin(intake, 0.75),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true).withTimeout(5),
                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true).withTimeout(5),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall_NC_LOW = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shooter, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new CMD_IntakeSpin(intake, 0.75),
                                                                        new CMD_ShooterRPM(shooter, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shooter, index, 2000),
                                                        new SEQ_dumbShot(shooter, index, 2000)),
                                        new CMD_canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall_NC_LOW_FullRun = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shooter, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new CMD_IntakeSpin(intake, 0.75),
                                                                        new CMD_ShooterRPM(shooter, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shooter, index, 2000),
                                                        new SEQ_dumbShot(shooter, index, 2000)),
                                        new CMD_canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                        new ParallelCommandGroup(
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IntakeSpin(intake, 0.75),
                                        new CMD_indexRun(index, 0.75),
                                        new CMD_ShooterRPM(shooter, 2000)).perpetually());

        Command RS_threeBall_WC_LOW = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shooter, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new CMD_IntakeSpin(intake, 0.75),
                                                                        new CMD_ShooterRPM(shooter, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shooter, index, 2000),
                                                        new SEQ_dumbShot(shooter, index, 2000)),
                                        new CMD_canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        // ====================================================================
        // Four ball command groups
        // ====================================================================
        Command RS_fourball = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_fourBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_fourBall_p1),
                                        new ParallelCommandGroup(
                                                        new CMD_ShooterRPM(shooter, 4500),
                                                        new SequentialCommandGroup(
                                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                                        new CMD_IndexBottomToTop(canal, index)))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_fourBall_p2),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(1),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_fourBall_p3),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        // ====================================================================
        // Defensive command groups
        // ====================================================================
        Command LS_blilliards = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_Billiards_p1.getInitialPose())),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(3),
                                        new CMD_canalThrough(canal, -1),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(0.5),
                                                        new CMD_indexRun(index, -0.75))),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_Billiards_p1),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(LS_Billiards_p2),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(3),
                                        new CMD_canalThrough(canal, -1),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(0.5),
                                                        new CMD_indexRun(index, -0.75))),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_citrus = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_Citrus_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(true), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_Citrus_p1),
                                        new CMD_IntakeSpin(intake, 0.75),
                                        new CMD_canalRun(canal, -0.75),
                                        new CMD_IndexBottomToTopBanner(index, 0.50)),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new CMD_canalThrough(canal, -1)),
                        new ParallelCommandGroup(
                                        new CMD_canalThrough(canal, -1),
                                        new CMD_indexRun(index, -0.75)));

        Command LS_defensive = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_Defensive_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_Defensive_p1),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new ParallelDeadlineGroup(
                                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                                        new CMD_canalRun(canal, -0.75)),
                        new SEQ_limeShot(shooter, drivetrain, index, limelight, true),
                        new InstantCommand(() -> intake.pistonSet(true), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_Defensive_p2),
                                        new CMD_IntakeSpin(intake, 0.75),
                                        new SequentialCommandGroup(
                                                        new CMD_CanalZeroToOneBottom(canal, index),
                                                        new CMD_IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new CMD_canalThrough(canal, -1),
                                        new CMD_IntakeSpin(intake, -0.75)),
                        new ParallelCommandGroup(
                                        new CMD_canalThrough(canal, -1),
                                        new CMD_indexRun(index, -0.75),
                                        new CMD_IntakeSpin(intake, -0.75)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                LiveWindow.disableAllTelemetry();
                configureButtonBindings();
                sendBallColor();

                CameraServer.startAutomaticCapture();

                limelight.setLed(1);
                field2d.getObject("traj").setTrajectory(RS_threeBall_p1);

                // AutoChooser.addOption("staright weaver", straightWeaver);
                AutoChooser.setDefaultOption("Low Dump - one ball - with drive", lowDump);
                AutoChooser.addOption("Low Dump - one ball - no drive", lowDumpNoDrive);
                AutoChooser.addOption("Drive Back - no shoot", straightAuto);
                AutoChooser.addOption("limelight - one ball - high shot - no drive", limelightHighShot);
                AutoChooser.addOption("Right side - two ball - Right Ball", RS_RB_twoBall);
                AutoChooser.addOption("Right side - two ball - Left Ball", RS_LB_twoBall);
                AutoChooser.addOption("Right side - three ball - high", RS_threeBall_NC_HIGH);
                // chooser.addOption("Right side three ball With Cam LOW", RS_threeBall_WC_LOW);
                AutoChooser.addOption("Right side - three ball - low", RS_threeBall_NC_LOW);
                AutoChooser.addOption("Right side - three ball - run end", RS_threeBall_NC_LOW_FullRun);
                AutoChooser.addOption("Right side - four ball ", RS_fourball);
                AutoChooser.addOption("Left side - two ball - No Cam", LS_twoBall_NC);
                AutoChooser.addOption("Left side - two ball - With Cam", LS_twoBall_WC);
                AutoChooser.addOption("Left side - Billiards - Defensive ", LS_blilliards);
                AutoChooser.addOption("Left side - Citrus - Defensive ", LS_citrus);
                AutoChooser.addOption("Left side - 2 Ball - Defensive ", LS_defensive);

                DelayChooser.setDefaultOption("0 sec", 0);
                DelayChooser.addOption("1 sec", 1);
                DelayChooser.addOption("2 sec", 2);
                DelayChooser.addOption("3 sec", 3);
                DelayChooser.addOption("4 sec", 4);
                DelayChooser.addOption("5 sec", 5);
                DelayChooser.addOption("6 sec", 6);
                DelayChooser.addOption("7 sec", 7);
                DelayChooser.addOption("8 sec", 8);
                DelayChooser.addOption("9 sec", 9);
                DelayChooser.addOption("10 sec", 10);

                SmartDashboard.putData("Auto Chooser", AutoChooser);
                SmartDashboard.putData("Delay Chooser", DelayChooser);


                // networkTables.start();
                System.out.println("RobotContainer initialization complete.");

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // drivetrain
                drivetrain.setDefaultCommand(new CMD_teleopDrive(drivetrain, () -> leftJoystick.getRawAxis(1),
                                () -> rightJoystick.getRawAxis(1)));
                L_button2.whenPressed(new InstantCommand(drivetrain::toggleReverse, drivetrain));

                // climber
                C_leftTrigger = new Trigger(() -> (controller.getRawAxis(2) > 0.5));
                C_rightTrigger = new Trigger(() -> (controller.getRawAxis(3) > 0.5));

                C_leftTrigger.whileActiveContinuous(new CMD_ClimberSpeed(climber, 1));
                C_rightTrigger.whileActiveContinuous(new CMD_ClimberSpeed(climber, -1));

                // Intake
                intakeDown.whileActiveContinuous(new CMD_AutoIntake(canal,intake,index));
                L_button4.whenPressed(new InstantCommand(intake::pistonToggle, intake));
                L_Trigger.whileHeld(new ParallelCommandGroup(new CMD_IntakeSpin(intake, 0.75),
                                new CMD_CanalZeroToOneBottom(canal, index)));

                // Canal
                C_dPadUp.whileHeld(new CMD_teleopCanalRun(canal, -0.75));
                C_dPadDown.whileHeld(new CMD_teleopCanalRun(canal, 0.75));
                C_dPadLeft.whileHeld(new CMD_teleopCanalThrough(canal, 0.75));
                C_dPadRight.whileHeld(new CMD_teleopCanalThrough(canal, -0.75));

                // Index
                indexBottomToTopTrigger.whileActiveContinuous(new CMD_IndexBottomToTop(canal, index));
                C_aButton.whileHeld(new ParallelCommandGroup(new CMD_indexRun(index, -0.75),
                                new CMD_ShooterSpin(shooter, 0.25)));
                C_bButton.whileHeld(new CMD_indexRun(index, 0.75));
                L_button5.whileHeld(new CMD_indexRun(index, 0.75));

                // shooter
                R_button3.whenPressed(new CMD_changeSetpoint(shooter, -500));
                R_button4.whenPressed(new CMD_changeSetpoint(shooter, -100));
                R_button5.whenPressed(new CMD_changeSetpoint(shooter, 500));
                R_button6.whenPressed(new CMD_changeSetpoint(shooter, 100));
                R_trigger.whileHeld(new CMD_ShooterManualRPM(shooter));

                // limelight
                limelight.setDefaultCommand(new InstantCommand(() -> limelight.setLed(1), limelight).perpetually());
                L_button3.whileHeld(new SEQ_limeShot(shooter, drivetrain, index, limelight, limelight.getHeight()));
                C_yButton.whenPressed(new InstantCommand(limelight::toggleHeight, limelight));

                // Ball Camera
                L_button10.whenPressed(cameraData::toggleDirection, cameraData);
                L_button11.whileHeld(new SEQ_getBall(cameraData, drivetrain, canal, intake, index,
                                cameraData.getDirection()));

                // LED
                LED.setDefaultCommand(new CMD_SOLIDLED(LED));

                // Color Sensor
                acceptBallTrigger.whenActive(new CMD_AcceptAllianceBall(canal, index, colorSensor));
                rejectBallTrigger.whenActive(new CMD_CanalRejectBall(colorSensor, canal, -0.75));
                rescindBallTrigger.whenActive(new CMD_RescindAllianceBall(index, canal,colorSensor));
                colorSensor.setDefaultCommand(new CMD_ManageBallQueue(colorSensor));
        }

        public Command getAutonomousCommand() {
                Command chosenAuto = AutoChooser.getSelected();
                int delay = DelayChooser.getSelected();
                drivetrain.zeroEncoders();
                drivetrain.zeroHeading();
                return new SequentialCommandGroup(new WaitCommand(delay), chosenAuto);
        }

        public void teleInit() {
                cameraData.setDirection(true);
                canal.setSpeedBack(0);
                canal.setSpeedFront(0);
        }

        public void teleopPeroid() {
                SmartDashboard.putBoolean("cam takeover", ((cameraData.getY() <= 40) && (cameraData.getY() >= 10)));
        }

        public static void sendBallColor() {
                var color = DriverStation.getAlliance();
                // send color to ball detection
                if (color == DriverStation.Alliance.Red) {
                        SmartDashboard.putString("ballColor", "red");

                } else if (color == DriverStation.Alliance.Blue) {
                        SmartDashboard.putString("ballColor", "blue");
                }
        }
}
