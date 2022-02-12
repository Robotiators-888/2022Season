// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.opencv.ml.StatModel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.teleopDrive;
import frc.robot.commands.teleopIndex;
import frc.robot.commands.zeroHeading;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.UDP.BallDataPacket;
import frc.robot.UDP.GenericBuffer;
import frc.robot.UDP.LimelightDataPacket;
import frc.robot.UDP.UDPReciever;
import frc.robot.commands.Aim;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.OuttakeMotorTest;
import frc.robot.commands.IntakeMotorTest;
import frc.robot.commands.PistonOutCmd;
import frc.robot.commands.PistonInCmd;



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

  // public final static Drivetrain m_driveSubsystem = new Drivetrain();
  private final GenericBuffer<BallDataPacket> ballBuffer = new GenericBuffer<>();
  private final GenericBuffer<LimelightDataPacket> limelightBuffer = new GenericBuffer<>();
  private final UDPReciever<BallDataPacket> m_BallReciever = new UDPReciever<>(Constants.BALL_PORT,
      () -> new BallDataPacket(), ballBuffer);
  private final UDPReciever<LimelightDataPacket> m_limelightReciever = new UDPReciever<>(Constants.LIMELIGHT_PORT,
      () -> new LimelightDataPacket(), limelightBuffer);
  private final Joystick m_stick = new Joystick(Constants.JOYSTICK_PORT);
  private Limelight m_limelight = new Limelight();
  private Shooter shoot = new Shooter();
  private Index m_index = new Index();

  private IntakeSubsystem m_intake = new IntakeSubsystem();

  private ColorSensorSubsystem colorSensor = new ColorSensorSubsystem();
  private IndexSubsystem index = new IndexSubsystem(colorSensor);


  private final Field2d field2d = new Field2d();
  private Drivetrain drivetrain = new Drivetrain(field2d);

  private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

  JoystickButton aButton = new JoystickButton(joystick, 1);
  JoystickButton bButton = new JoystickButton(joystick, 2);
  JoystickButton cButton = new JoystickButton(joystick, 3);

  JoystickButton yButton = new JoystickButton(joystick, 4);
  JoystickButton leftShoulder = new JoystickButton(joystick, 5);
  JoystickButton rightShoulder = new JoystickButton(joystick, 6);
  JoystickButton thumbLeft = new JoystickButton(joystick, 7);
  JoystickButton thumbRight = new JoystickButton(joystick, 8);




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_BallReciever.start();
    m_limelightReciever.start();

    // periodic getting
    // separate function > getting string value

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
    cButton.whenPressed(new zeroHeading(drivetrain));
    thumbLeft.whenPressed(new PistonInCmd(m_intake));
    thumbRight.whenPressed(new PistonOutCmd(m_intake));

    // While a button is pressed, run autoshoot command
    aButton.whileHeld(new LimelightCommand(m_limelight, shoot, m_index));
    // While b button is pressed, run autoaim command
    bButton.whileHeld(new Aim(m_limelight, drivetrain));

    leftShoulder.whileHeld(new IntakeMotorTest(m_intake));
    rightShoulder.whileHeld(new OuttakeMotorTest(m_intake));


    yButton.whileHeld(new teleopIndex(index));
  }

  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 2, new Rotation2d(0)),
        List.of(), new Pose2d(5, 2, new Rotation2d(0)), config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts, drivetrain);

    field2d.getObject("traj").setTrajectory(exampleTrajectory);

    drivetrain.zeroHeading();
    drivetrain.setPosition(2, 2, new Rotation2d(0));

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

}

/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
