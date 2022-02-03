// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FMSCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FMSSubsystem;
import frc.robot.subsystems.UDPRecieverSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


import frc.robot.commands.UDPReceiverCmd;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final UDPRecieverSubsystem m_udpsubsystem = new UDPRecieverSubsystem();
  public final static FMSSubsystem m_fmssubsystem = new FMSSubsystem();
  public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  Joystick m_stick = new Joystick(Constants.JOYSTICK_PORT);
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
    m_fmssubsystem.setDefaultCommand(new FMSCmd(m_fmssubsystem));
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }


  private double applyDeadZone(double axisVal){
    double dz = Constants.DEAD_ZONE;
    if (axisVal>dz && axisVal<-dz){
      return 0;
    }
    return axisVal;
  }
}
