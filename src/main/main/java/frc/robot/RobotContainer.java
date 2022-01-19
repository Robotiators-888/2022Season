// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;



import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.commands.ColorSensorCommand;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Joystick;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ColorSensorSubsystem m_colorSubsystem = new ColorSensorSubsystem();
  private final Joystick m_stick = new Joystick(Constants.JOYSTICK_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    


    ColorSensorCommand colorSensor = new ColorSensorCommand(m_colorSubsystem);
    m_colorSubsystem.setDefaultCommand(colorSensor);
    
// periodic getting
// separate function > getting string value
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
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
