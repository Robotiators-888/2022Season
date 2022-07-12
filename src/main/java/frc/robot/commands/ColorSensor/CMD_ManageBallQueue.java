// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ColorSensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SUB_ColorSensor;
import frc.robot.Constants;

import java.util.function.Supplier;

public class CMD_ManageBallQueue extends CommandBase {

  private SUB_ColorSensor colorSensor;
  
  private Alliance prevFr = Alliance.Invalid;
  private Supplier<Alliance> frBallType;
  
  
  /** Creates a new CMD_CanalRejectionSystem. */
  public CMD_ManageBallQueue(SUB_ColorSensor colorSensorArgs) {
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
    frBallType = ()->colorSensor.readSensor(Constants.FRONT_COLOR_SENSOR_ID);

    SmartDashboard.putString("Color", colorSensor.allianceToColor(frBallType.get()));
    SmartDashboard.putString("Queue", colorSensor.allianceToColor(colorSensor.peekQ()));
    
    if (!colorSensor.isUnknown(frBallType.get()) && frBallType.get()!=prevFr) {
      colorSensor.pushQ(frBallType.get());
    } 
    
    prevFr = frBallType.get();

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
