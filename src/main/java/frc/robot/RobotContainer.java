// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BallCam.SEQ_getBall;
import frc.robot.commands.LimeLight.*;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.subsystems.SUB_LED;
import frc.robot.commands.LEDPatterns.*;

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
        private Shooter shoot = new Shooter();
        private Drivetrain drivetrain = new Drivetrain(field2d);
        private IntakeSubsystem intake = new IntakeSubsystem();
        private IndexSubsystem index = new IndexSubsystem();
        private Autonomous autoHelper = new Autonomous(drivetrain);
        private CanalSubsystem canal = new CanalSubsystem();
        private Climber climber = new Climber();
        private Limelight limelight = new Limelight();
        private SUB_CameraData cameraData = new SUB_CameraData();
        private SUB_LED LED = new SUB_LED();

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
        SendableChooser<Command> chooser = new SendableChooser<>();
        TrajectoryConfig configReversed = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .setReversed(true).setEndVelocity(0.2);

        TrajectoryConfig configForward = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);

        Trajectory LS_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p1.wpilib.json");
        Trajectory LS_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p2.wpilib.json");

        Trajectory RS_RB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_RB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_LB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_LB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_threeBall_p1 = autoHelper.getTrajectory("paths/output/RS_threeBall_p1.wpilib.json");
        Trajectory RS_threeBall_p2 = autoHelper.getTrajectory("paths/output/RS_threeBall_p2_v2.wpilib.json");

        Trajectory RS_threeBall_p2_LOW = autoHelper.getTrajectory("paths/output/RS_threeBall_p2_LOW.wpilib.json");

        Trajectory Str8 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180))),
                        List.of(), new Pose2d(4, 0, new Rotation2d(Units.degreesToRadians(180))), configReversed);

        // Auto command groups
        Command limelightHighShot = new SequentialCommandGroup(
                        new SEQ_limeShot(shoot, drivetrain, index, limelight, true));

        Command straightAuto = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        autoHelper.getRamset(Str8));

        Command lowDump = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new SEQ_dumbShot(shoot, index, 1800),
                        autoHelper.getRamset(Str8));

        Command lowDumpNoDrive = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new SEQ_dumbShot(shoot, index, 1800));

        Command RS_RB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_RB_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_RB_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_RB_twoBall_Low_p2),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_LB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_LB_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_LB_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_LB_twoBall_Low_p2),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_twoBall_NC = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_twoBall_Low_p1.getInitialPose())),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(LS_twoBall_Low_p2),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_twoBall_WC = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_twoBall_Low_p1.getInitialPose())),
                        new InstantCommand(() -> cameraData.setDirection(false), cameraData),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new ParallelRaceGroup(
                                        autoHelper.getRamset(LS_twoBall_Low_p1),

                                        new SequentialCommandGroup(
                                                        new InstantCommand().withInterrupt(
                                                                        () -> ((cameraData.getY() <= 45)
                                                                                        && (cameraData.getY() >= 10)
                                                                                        || (Math.abs(cameraData
                                                                                                        .getX()) > 3)
                                                                                                        && (cameraData.getY() <= 30))),
                                                        new SEQ_getBall(cameraData, drivetrain, canal, intake, index,
                                                                        false).withTimeout(6))));
        // .andThen(new SEQ_getBall(cameraData, drivetrain, canal, intake, index,
        // false).withTimeout(6))
        // new ParallelDeadlineGroup(
        // autoHelper.getRamset(LS_twoBall_Low_p1).withInterrupt(
        // () -> ((cameraData.getY() <= 45) && (cameraData.getY() >= 10)
        // || (Math.abs(cameraData.getX()) > 3)
        // && (cameraData.getY() <= 30)))));
        // new canalRun(canal, -0.75),
        // new IndexBottomToTopBanner(index, 0.50)),
        // //new InstantCommand(() -> drivetrain.setMotors(0, 0), drivetrain),
        // new SEQ_getBall(cameraData, drivetrain, canal, intake, index,
        // false).withTimeout(6),
        // new ParallelDeadlineGroup(
        // new WaitCommand(2),
        // new SequentialCommandGroup(
        // new CanalZeroToOneBottom(canal, index),
        // new IndexBottomToTop(canal, index))),
        // autoHelper.getRamset(LS_twoBall_Low_p2),
        // new SEQ_dumbShot(shoot, index, 1800),
        // new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall_NC_HIGH = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new SEQ_dumbShot(shoot, index, 1800),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p1),
                                        new SequentialCommandGroup(
                                                        new CanalZeroToOneBottom(canal, index),
                                                        new IndexBottomToTop(canal, index))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(5),
                                        new SequentialCommandGroup(
                                                        new CanalZeroToOneBottom(canal, index),
                                                        new IndexBottomToTop(canal, index))),
                        new InstantCommand(() -> intake.pistonSet(true), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p2),
                                        new IntakeSpin(intake, 0.75),
                                        new SequentialCommandGroup(
                                                        new CanalZeroToOneBottom(canal, index),
                                                        new IndexBottomToTop(canal, index))),
                        new SEQ_limeShot(shoot, drivetrain, index, limelight, true).withTimeout(5),
                        new SEQ_limeShot(shoot, drivetrain, index, limelight, true).withTimeout(5),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall_NC_LOW = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shoot, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new IntakeSpin(intake, 0.75),
                                                                        new ShooterRPM(shoot, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shoot, index, 2000),
                                                        new SEQ_dumbShot(shoot, index, 2000)),
                                        new canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall_NC_LOW_FullRun = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shoot, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new IntakeSpin(intake, 0.75),
                                                                        new ShooterRPM(shoot, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shoot, index, 2000),
                                                        new SEQ_dumbShot(shoot, index, 2000)),
                                        new canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                        new ParallelCommandGroup(
                                        new canalRun(canal, -0.75),
                                        new IntakeSpin(intake, 0.75),
                                        new indexRun(index, 0.75),
                                        new ShooterRPM(shoot, 2000)).perpetually());

        Command RS_threeBall_WC_LOW = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                        new SEQ_dumbShot(shoot, index, 1800),
                                                        autoHelper.getRamset(RS_threeBall_p1),
                                                        new WaitCommand(1),
                                                        new InstantCommand(() -> intake.pistonSet(true), intake),
                                                        new ParallelDeadlineGroup(
                                                                        autoHelper.getRamset(RS_threeBall_p2_LOW),
                                                                        new IntakeSpin(intake, 0.75),
                                                                        new ShooterRPM(shoot, 2000)),
                                                        new InstantCommand(
                                                                        () -> intake.pistonSet(false),
                                                                        intake),
                                                        new SEQ_dumbShot(shoot, index, 2000),
                                                        new SEQ_dumbShot(shoot, index, 2000)),
                                        new canalRun(canal, -0.75)),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                LiveWindow.disableAllTelemetry();
                configureButtonBindings();
                sendBallColor();

                limelight.setLed(1);
                field2d.getObject("traj").setTrajectory(RS_threeBall_p1);

                chooser.setDefaultOption("Low Dump", lowDump);
                chooser.addOption("limelight High Shot", limelightHighShot);
                chooser.addOption("Low Dump no drive", lowDumpNoDrive);
                chooser.addOption("Drive Back", straightAuto);
                chooser.addOption("Right side Right Ball", RS_RB_twoBall);
                chooser.addOption("Right side Left Ball", RS_LB_twoBall);
                chooser.addOption("Left side - No Cam", LS_twoBall_NC);
                chooser.addOption("Left side - With Cam", LS_twoBall_WC);
                chooser.addOption("Right side three ball - No Cam - HIGH", RS_threeBall_NC_HIGH);
                // chooser.addOption("Right side three ball With Cam LOW", RS_threeBall_WC_LOW);
                chooser.addOption("Right side three ball - No cam - LOW", RS_threeBall_NC_LOW);
                chooser.addOption("three ball run end", RS_threeBall_NC_LOW_FullRun);

                SmartDashboard.putData("chooser", chooser);

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
                IndexBottomToTop DefCMD = new IndexBottomToTop(canal, index);

                // drivetrain
                drivetrain.setDefaultCommand(new teleopDrive(drivetrain, () -> leftJoystick.getRawAxis(1),
                                () -> rightJoystick.getRawAxis(1)));
                L_button2.whenPressed(new InstantCommand(drivetrain::toggleReverse, drivetrain));

                // climber
                C_leftTrigger = new Trigger(() -> (controller.getRawAxis(2) > 0.5));
                C_rightTrigger = new Trigger(() -> (controller.getRawAxis(3) > 0.5));

                C_leftTrigger.whileActiveContinuous(new teleopClimber(climber, 0.85));
                C_rightTrigger.whileActiveContinuous(new teleopClimber(climber, -0.85));

                // Intake
                L_button4.whenPressed(new InstantCommand(intake::pistonToggle, intake));
                L_Trigger.whileHeld(new ParallelCommandGroup(new IntakeSpin(intake, 0.75),
                                new CanalZeroToOneBottom(canal, index)));

                // Canal
                C_dPadUp.whileHeld(new canalRun(canal, -0.75));
                C_dPadDown.whileHeld(new canalRun(canal, 0.75));
                C_dPadLeft.whileHeld(new CMD_canalThrough(canal, 0.75));
                C_dPadRight.whileHeld(new CMD_canalThrough(canal, -0.75));

                // Index
                index.setDefaultCommand(DefCMD);
                C_aButton.whileHeld(new ParallelCommandGroup(new indexRun(index, -0.75), new ShooterSpin(shoot, 0.25)));
                C_bButton.whileHeld(new indexRun(index, 0.75));
                L_button5.whileHeld(new indexRun(index, 0.75));

                // shooter
                R_button3.whenPressed(new CMD_changeSetpoint(shoot, -500));
                R_button4.whenPressed(new CMD_changeSetpoint(shoot, -100));
                R_button5.whenPressed(new CMD_changeSetpoint(shoot, 500));
                R_button6.whenPressed(new CMD_changeSetpoint(shoot, 100));
                R_trigger.whileHeld(new CMD_ShooterManualRPM(shoot));

                // limelight
                limelight.setDefaultCommand(new InstantCommand(() -> limelight.setLed(1), limelight).perpetually());
                L_button3.whileHeld(new SEQ_limeShot(shoot, drivetrain, index, limelight, true));
                // C_yButton.whenPressed(new InstantCommand(limelight::toggleHeight,
                // limelight));
                L_button10.whenPressed(cameraData::toggleDirection, cameraData);
                L_button11.whileHeld(new SEQ_getBall(cameraData, drivetrain, canal, intake, index,
                                cameraData.getDirection()));

                // LED
                LED.setDefaultCommand(new CMD_SOLIDLED(LED));
        }

        public Command getAutonomousCommand() {
                drivetrain.zeroEncoders();
                drivetrain.zeroHeading();
                return chooser.getSelected();
        }

        public void teleInit() {
                cameraData.setDirection(true);
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
