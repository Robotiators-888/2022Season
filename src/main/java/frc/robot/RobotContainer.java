// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleopDrive;
import frc.robot.commands.indexRun;
import frc.robot.commands.zeroHeading;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.UDP.BallDataPacket;
import frc.robot.UDP.GenericBuffer;
import frc.robot.UDP.LimelightDataPacket;
import frc.robot.UDP.UDPReciever;
import frc.robot.commands.Aim;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.OuttakeMotorTest;
import frc.robot.commands.IntakeMotorTest;
import frc.robot.commands.PistonOutCmd;
import frc.robot.commands.PistonInCmd;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private final UDPReciever<BallDataPacket> m_BallReciever = new UDPReciever<>(Constants.BALL_PORT,
            () -> new BallDataPacket(), ballBuffer);
    private final UDPReciever<LimelightDataPacket> m_limelightReciever = new UDPReciever<>(Constants.LIMELIGHT_PORT,
            () -> new LimelightDataPacket(), limelightBuffer);

    private final Field2d field2d = new Field2d();

    // subsystems
    private Limelight m_limelight = new Limelight();
    private Shooter shoot = new Shooter();
    private Drivetrain drivetrain = new Drivetrain(field2d);
    private IntakeSubsystem m_intake = new IntakeSubsystem();
    private IndexSubsystem index = new IndexSubsystem();
    private Autonomous autoHelper = new Autonomous(drivetrain);
    private CanalSubsystem canal = new CanalSubsystem();

    // Joystick objects
    private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    private Joystick twiststick = new Joystick(Constants.TWISTSTICK_PORT);
    private Joystick leftJoystick = new Joystick(Constants.LEFTJOYSTICK_PORT);

    JoystickButton aButton = new JoystickButton(joystick, 1);
    JoystickButton bButton = new JoystickButton(joystick, 2);
    JoystickButton xButton = new JoystickButton(joystick, 3);
    JoystickButton yButton = new JoystickButton(joystick, 4);
    JoystickButton leftShoulder = new JoystickButton(joystick, 5);
    JoystickButton rightShoulder = new JoystickButton(joystick, 6);
    JoystickButton backButton = new JoystickButton(joystick, 7);
    JoystickButton startButton = new JoystickButton(joystick, 8);
    JoystickButton thumbLeft = new JoystickButton(joystick, 9);
    JoystickButton thumbRight = new JoystickButton(joystick, 10);

    POVButton dPadUp = new POVButton(joystick, 0);
    POVButton dPadDown = new POVButton(joystick, 180);


    JoystickButton button7 = new JoystickButton(leftJoystick, 7);

    // Auto objects
    SendableChooser<Command> chooser = new SendableChooser<>();
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);
    Trajectory ballin1 = autoHelper.getTrajectory("paths/test2balll.wpilib.json");
    Trajectory ballin2 = autoHelper.getTrajectory("paths/test2balll_0.wpilib.json");
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        
        field2d.getObject("traj").setTrajectory(Str8);

        chooser.setDefaultOption("Simple Auto", straightAuto);
        chooser.addOption("Complex Auto", pwtest);

        m_BallReciever.start();
        m_limelightReciever.start();
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
        drivetrain.setDefaultCommand(new teleopDrive(drivetrain, () -> joystick.getRawAxis(Constants.LEFT_AXIS),
                () -> joystick.getRawAxis(Constants.RIGHT_AXIS)));
        //xButton.whenPressed(new zeroHeading(drivetrain));
        startButton.whileHeld(new Aim(m_limelight, drivetrain));
        aButton.whileHeld(new ParallelCommandGroup(
                                new ShooterSpin(shoot, twiststick, Constants.ShooterSpeed),
                                new SequentialCommandGroup(
                                        new WaitCommand(2),
                                        new ParallelCommandGroup(
                                                new indexRun(index,Constants.BELT_SPEED), 
                                                new canalRun(canal,-Constants.BELT_SPEED))
                                )));


        //shooter controls

        //intake Controls
        leftShoulder.whileHeld(new IntakeSpin(m_intake, 0.75));
        rightShoulder.whileHeld(new IntakeSpin(m_intake, -0.75));
        bButton.whenPressed(new InstantCommand(() -> m_intake.pistonToggle()));
       
        
        //spins shooter backwards
        button7.toggleWhenPressed(new ShooterSpin(shoot, twiststick, 0.50));


        //aButton.whileHeld(new ShooterSpin(shoot, twiststick));
        //bButton.whileHeld(new IntakeMotorTest(m_intake));

       // xButton.whenPressed(new zeroHeading(drivetrain));
        yButton.whileHeld(new ParallelCommandGroup(new indexRun(index,Constants.BELT_SPEED), new canalRun(canal,-Constants.BELT_SPEED)));
        xButton.whileHeld(new ParallelCommandGroup(new indexRun(index,-Constants.BELT_SPEED), new canalRun(canal,Constants.BELT_SPEED)));
        //xButton.whileHeld (new OrganizeIndexCMD(index));

        dPadUp.whileHeld(new ParallelCommandGroup(new canalRun(canal,-Constants.BELT_SPEED),new IntakeSpin(m_intake, 0.75)));
        dPadDown.whileHeld(new ParallelCommandGroup(new canalRun(canal,Constants.BELT_SPEED),new IntakeSpin(m_intake, -0.75)));
}

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}