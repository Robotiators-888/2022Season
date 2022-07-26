// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ColorSensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_ColorSensor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_ColorOnDashboard extends CommandBase {
  private SUB_ColorSensor colorSensor;
  /** Creates a new CMD_ColorOnDashboard. */
  public CMD_ColorOnDashboard(SUB_ColorSensor colorSensorArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    colorSensor = colorSensorArgs;
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String color;
    switch (colorSensor.readSensor(0)){
      case Red:
        color = "Red";
        break;
      case Blue:
        color="Blue";
        break;
      default: color="Unknown"; break;
    }

    SmartDashboard.putString("Color",color);
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
