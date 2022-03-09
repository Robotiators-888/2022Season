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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.NetworkTables.NetworkTablesBase;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
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
        private SUB_LED LED = new SUB_LED();
        private NetworkTablesBase networkTables = new NetworkTablesBase();
        //private NetworkTablesBase networkTables = new NetworkTablesBase();

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
                                        .setReversed(true);

        Trajectory LS_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p1.wpilib.json");
        Trajectory LS_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/LS_twoBall_Low_p2.wpilib.json");

        Trajectory RS_RB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_RB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_RB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_LB_twoBall_Low_p1 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p1.wpilib.json");
        Trajectory RS_LB_twoBall_Low_p2 = autoHelper.getTrajectory("paths/output/RS_LB_twoBall_Low_p2.wpilib.json");

        Trajectory RS_threeBall_p1 = autoHelper.getTrajectory("paths/output/RS_threeBall_p1.wpilib.json");
        Trajectory RS_threeBall_p2 = autoHelper.getTrajectory("paths/output/RS_threeBall_p2_v2.wpilib.json");

        Trajectory Str8 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180))),
                        List.of(), new Pose2d(4, 0, new Rotation2d(Units.degreesToRadians(180))), configReversed);

        // Auto command groups
        Command straightAuto = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        autoHelper.getRamset(Str8));

        Command lowDump = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        autoHelper.getRamset(Str8));

        Command lowDumpNoDrive = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))));

        Command RS_RB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_RB_twoBall_Low_p1.getInitialPose())),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_RB_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_RB_twoBall_Low_p2),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_LB_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_LB_twoBall_Low_p1.getInitialPose())),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_LB_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(RS_LB_twoBall_Low_p2),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command LS_twoBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(LS_twoBall_Low_p1.getInitialPose())),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(LS_twoBall_Low_p1),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        autoHelper.getRamset(LS_twoBall_Low_p2),
                        new ParallelRaceGroup(
                                        new ShooterRPM(shoot, 2000),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new indexRun(index, 0.75).withTimeout(2))),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        Command RS_threeBall = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(RS_threeBall_p1.getInitialPose())),
                        new AutoShoot(limelight, index, drivetrain, shoot).withInterrupt(() -> !index.readTopBanner()),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p1),
                                        new SequentialCommandGroup(
                                                        new CanalZeroToOneBottom(canal, index),
                                                        new IndexBottomToTopBanner(index, 0.50))),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new SequentialCommandGroup(
                                                        new CanalZeroToOneBottom(canal, index),
                                                        new IndexBottomToTopBanner(index, 0.50))),
                        new AutoShoot(limelight, index, drivetrain, shoot).withInterrupt(() -> !index.readTopBanner()),
                        new InstantCommand(() -> intake.pistonSet(false), intake),
                        new ParallelDeadlineGroup(
                                        autoHelper.getRamset(RS_threeBall_p2),
                                        new IntakeSpin(intake, 0.75),
                                        new canalRun(canal, -0.75),
                                        new IndexBottomToTopBanner(index, 0.50)),
                        new AutoShoot(limelight, index, drivetrain, shoot).withInterrupt(() -> !index.readTopBanner()),

                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                LiveWindow.disableAllTelemetry();
                configureButtonBindings();

                limelight.setLed(1);
                // field2d.getObject("traj").setTrajectory(Str8);

                chooser.setDefaultOption("Low Dump", lowDump);
                chooser.addOption("Low Dump no drive", lowDumpNoDrive);
                chooser.addOption("Drive Back", straightAuto);
                chooser.addOption("Right side Right Ball", RS_RB_twoBall);
                chooser.addOption("Right side Left Ball", RS_LB_twoBall);
                chooser.addOption("Left side", LS_twoBall);
                chooser.addOption("Right side three ball", RS_threeBall);

                SmartDashboard.putData("chooser", chooser);

                networkTables.start();
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

                C_leftTrigger.whileActiveContinuous(new teleopClimber(climber, 0.50));
                C_rightTrigger.whileActiveContinuous(new teleopClimber(climber, -0.50));

                
                //intake.setDefaultCommand(new ConditionalCommand(new ParallelCommandGroup(new IntakeSpin(intake, 0.75),new CanalZeroToOneBottom(canal, index)), new InstantCommand(), intake::intakeGet));
                L_button4.whenPressed(new InstantCommand(intake::pistonToggle, intake));
                L_Trigger.whileHeld(new ParallelCommandGroup(new IntakeSpin(intake, 0.75),new CanalZeroToOneBottom(canal, index)));

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

                // L_button5.whileHeld(new CameraDriveCommand(drivetrain));
                // L_button5.whileHeld(new ParallelCommandGroup(new
                // CameraDriveCommand(drivetrain), new ParallelCommandGroup(new
                // IntakeSpin(intake, 0.75), new CanalZeroToOneBottom(canal, index))));
                L_button3.whileHeld(new AutoShoot(limelight, index, drivetrain, shoot));
                C_yButton.whenPressed(new InstantCommand(limelight::toggleHeight, limelight));

                //LED
                LED.setDefaultCommand(new CMD_HALFLED(LED));

        }

        public Command getAutonomousCommand() {
                drivetrain.zeroHeading();
                drivetrain.zeroEncoders();
                return chooser.getSelected();
        }

}