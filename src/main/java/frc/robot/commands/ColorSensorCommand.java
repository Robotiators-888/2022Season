// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ColorSensorSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ColorSensorCommand extends CommandBase {

  private ColorSensorSubsystem m_subsystem;
  String color;

  /** Creates a new ColorSensorCommand. */
  public ColorSensorCommand(ColorSensorSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String color = m_subsystem.findColor();
    double[] RGBArray = m_subsystem.findRGB();

    SmartDashboard.putString("Detected Color",color);

    SmartDashboard.putNumber("Red",RGBArray[0]);
    SmartDashboard.putNumber("Blue",RGBArray[1]);
    SmartDashboard.putNumber("Green",RGBArray[2]);
    SmartDashboard.putNumber("Sum RGB",RGBArray[0]+RGBArray[1]+RGBArray[2]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
