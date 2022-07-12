// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Canal.CSRejection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_ColorSensor;



public class CMD_CanalRejectBall extends CommandBase {

  private SUB_ColorSensor colorSensor;
  private SUB_Canal canal;
  private double speed;

  /** Creates a new CMD_CanalRejectBall. */
  public CMD_CanalRejectBall(SUB_ColorSensor colorSensorArgs, SUB_Canal canalArgs, double speedArgs) {
    // Use addRequirements() here to declare subsystem dependencies.

    colorSensor = colorSensorArgs;
    canal = canalArgs;
    speed = speedArgs;
    addRequirements(canal,colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canal.rejecting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canal.setSpeedBack(-speed);
    canal.setSpeedFront(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canal.setSpeedBack(0);
    canal.setSpeedFront(0);
    colorSensor.popQ();
    canal.rejecting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorSensor.isUnknown(colorSensor.peekQ()) || colorSensor.isOpp(colorSensor.readSensor(Constants.BACK_CANAL_ID));
  }
}
