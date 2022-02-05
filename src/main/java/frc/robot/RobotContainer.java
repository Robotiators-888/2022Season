// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.UDPRecieverSubsystem;



import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.commands.ColorSensorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


import frc.robot.commands.UDPReceiverCmd;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.teleopDrive;
import frc.robot.commands.zeroHeading;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final UDPRecieverSubsystem m_udpsubsystem = new UDPRecieverSubsystem();
  public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final ColorSensorSubsystem m_colorSubsystem = new ColorSensorSubsystem();
  private final Joystick m_stick = new Joystick(Constants.JOYSTICK_PORT);


  private final Field2d field2d = new Field2d();

  private Drivetrain drivetrain = new Drivetrain(field2d);

  private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

  JoystickButton AButton = new JoystickButton(joystick, 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriveCmd driveTrain = new DriveCmd(m_driveSubsystem,
    () -> applyDeadZone(m_stick.getRawAxis(Constants.RIGHT_AXIS)),
    ()-> applyDeadZone(m_stick.getRawAxis(Constants.LEFT_AXIS)));

    m_driveSubsystem.setDefaultCommand(driveTrain);
    m_udpsubsystem.setDefaultCommand(new UDPReceiverCmd(m_udpsubsystem));
    


    ColorSensorCommand colorSensor = new ColorSensorCommand(m_colorSubsystem);
    m_colorSubsystem.setDefaultCommand(colorSensor);
    
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
    AButton.whenPressed(new zeroHeading(drivetrain));
  }

  /*
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 2, new Rotation2d(0)),
        List.of(new Translation2d(4, 2)), new Pose2d(5, 2, new Rotation2d(0)), config);

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
  */

  private double applyDeadZone(double axisVal){
    double dz = Constants.DEAD_ZONE;
    if (axisVal>dz && axisVal<-dz){
      return 0;
    }
    return axisVal;
  }
}
