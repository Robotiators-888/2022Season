// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Aim;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Limelight m_limelight = new Limelight();
  private Shooter shoot = new Shooter();
  private Index m_index = new Index();
  private Drivetrain drive = new Drivetrain();
  
  private Joystick stick = new Joystick(0);
  private JoystickButton aButton = new JoystickButton(stick, 1);
  private JoystickButton bButton = new JoystickButton(stick, 2);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Default drive with controller joysticks
    drive.setDefaultCommand(new teleopDrive(drive, () -> stick.getRawAxis(Constants.LEFT_AXIS),
    () -> stick.getRawAxis(Constants.RIGHT_AXIS)));
    //While a button is pressed, run autoshoot command
    aButton.whileHeld(new LimelightCommand(m_limelight, shoot, m_index));
    //While b button is pressed, run autoaim command
    bButton.whileHeld(new Aim(m_limelight, drive));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
