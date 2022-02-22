// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleopDrive;
import frc.robot.commands.indexRun;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.CanalToBottomCMD; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.UDP.BallDataPacket;
import frc.robot.UDP.GenericBuffer;
import frc.robot.UDP.LimelightDataPacket;
import frc.robot.UDP.UDPReciever;
import frc.robot.commands.Aim;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.CMD_ShooterRPM;
import frc.robot.commands.CMD_canalThrough;
import frc.robot.commands.CMD_changeSetpoint;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.ShooterSpin;
import frc.robot.commands.canalRun;
import frc.robot.commands.teleopClimber;

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
        private final GenericBuffer<BallDataPacket> ballBuffer = new GenericBuffer<>();
        private final GenericBuffer<LimelightDataPacket> limelightBuffer = new GenericBuffer<>();
        private final UDPReciever<BallDataPacket> BallReciever = new UDPReciever<>(Constants.BALL_PORT,
                        () -> new BallDataPacket(), ballBuffer);
        private final UDPReciever<LimelightDataPacket> limelightReciever = new UDPReciever<>(Constants.LIMELIGHT_PORT,
                        () -> new LimelightDataPacket(), limelightBuffer);

        private final Field2d field2d = new Field2d();

        // subsystems
        private Limelight limelight = new Limelight();
        private Shooter shoot = new Shooter();
        private Drivetrain drivetrain = new Drivetrain(field2d);
        private IntakeSubsystem intake = new IntakeSubsystem();
        private IndexSubsystem index = new IndexSubsystem();
        private Autonomous autoHelper = new Autonomous(drivetrain);
        private CanalSubsystem canal = new CanalSubsystem();
        private Climber climber = new Climber();

        // Controller
        private Joystick controller = new Joystick(Constants.JOYSTICK_PORT);

        JoystickButton C_aButton = new JoystickButton(controller, 1);
        JoystickButton C_bButton = new JoystickButton(controller, 2);
        JoystickButton C_xButton = new JoystickButton(controller, 3);
        JoystickButton C_yButton = new JoystickButton(controller, 4);
        POVButton C_dPadUp = new POVButton(controller, 0);
        POVButton C_dPadDown = new POVButton(controller, 180);
        POVButton C_dPadLeft = new POVButton(controller, 270);
        POVButton C_dPadRight= new POVButton(controller, 90);
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
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);
        Trajectory ballin1 = autoHelper.getTrajectory("paths/output/test2balll.wpilib.json");
        Trajectory ballin2 = autoHelper.getTrajectory("paths/output/test2balll_0.wpilib.json");
        Trajectory onePath = autoHelper.getTrajectory("paths/output/onepathwonder.wpilib.json");
        Trajectory Str8 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(1, 0)), new Pose2d(2, 0, new Rotation2d(0)), config);

        // Auto command groups
        Command straightAuto = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        autoHelper.getRamset(Str8));

        Command pwtest = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(ballin1.getInitialPose())),
                        autoHelper.getRamset(ballin1),
                        autoHelper.getRamset(ballin2).andThen(() -> drivetrain.tankDriveVolts(0, 0)));

        Command onePathWonder = new SequentialCommandGroup(new ParallelRaceGroup(
                        new ParallelCommandGroup(new indexRun(index, Constants.BELT_SPEED),
                                        new canalRun(canal, -Constants.BELT_SPEED)),
                        new SequentialCommandGroup(
                                        new InstantCommand(() -> drivetrain.setPosition(onePath.getInitialPose())),
                                        autoHelper.getRamset(onePath).andThen(() -> drivetrain.tankDriveVolts(0, 0)))),
                        new ParallelCommandGroup(new indexRun(index, Constants.BELT_SPEED),
                                        new ShooterSpin(shoot, 0.50)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                LiveWindow.disableAllTelemetry();
                configureButtonBindings();

                field2d.getObject("traj").setTrajectory(Str8);

                chooser.setDefaultOption("Simple Auto", straightAuto);
                chooser.addOption("Complex Auto", pwtest);
                chooser.addOption("one Path Wonder", onePathWonder);

                BallReciever.start();
                limelightReciever.start();
                SmartDashboard.putData("chooser", chooser);
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
                //drivetrain
                drivetrain.setDefaultCommand(new teleopDrive(drivetrain, () -> leftJoystick.getRawAxis(1),
                                () -> rightJoystick.getRawAxis(1)));
                L_button2.whenPressed(new InstantCommand(drivetrain::toggleReverse, drivetrain));

                //climber                
                C_leftTrigger = new Trigger(() -> (controller.getRawAxis(2) > 0.5));
                C_rightTrigger = new Trigger(() -> (controller.getRawAxis(3) > 0.5));

                C_leftTrigger.whileActiveContinuous(new teleopClimber(climber, -0.25));
                C_rightTrigger.whileActiveContinuous(new teleopClimber(climber, 0.25));

                //Intake
                L_button4.whenPressed(new InstantCommand(intake::pistonToggle, intake));
                L_Trigger.whileHeld(new ParallelCommandGroup(new IntakeSpin(intake, 0.75), new canalRun(canal, -0.75)));

                //Canal
                C_dPadUp.whileHeld(new canalRun(canal, -0.75));
                C_dPadDown.whileHeld(new canalRun(canal, 0.75));
                C_dPadLeft.whileHeld(new CMD_canalThrough(canal, 0.75));
                C_dPadRight.whileHeld(new CMD_canalThrough(canal, -0.75));

                //Index
                C_aButton.whileHeld(new ParallelCommandGroup(new indexRun(index, -0.75), new ShooterSpin(shoot, 0.25)));
                C_bButton.whileHeld(new indexRun(index, 0.75));

                //shooter
                R_button3.whenPressed(new CMD_changeSetpoint(shoot, -500));
                R_button4.whenPressed(new CMD_changeSetpoint(shoot, -100));
                R_button5.whenPressed(new CMD_changeSetpoint(shoot, 500));
                R_button6.whenPressed(new CMD_changeSetpoint(shoot, 100));
                R_trigger.whileHeld(new CMD_ShooterRPM(shoot, shoot.getManualRPM()));
                //L_button2 auto aim and shoot
                L_button2.whileHeld(new SequentialCommandGroup(new Aim(limelight, drivetrain), new AutoShoot(limelight, shoot, index)));
                //C_yButton change auto target (high or low goal)
                C_yButton.whenPressed(new InstantCommand(limelight::toggleHeight, limelight));

        }

        public Command getAutonomousCommand() {
                return chooser.getSelected();
        }

}