// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Canal.CSRejection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_ColorSensor;

public class CMD_AcceptAllianceBall extends CommandBase {

  private SUB_Canal canal;
  private SUB_Index index;
  private SUB_ColorSensor colorSensor;

  /** Creates a new CMD_AcceptAllianceBall. */
  public CMD_AcceptAllianceBall(SUB_Canal canalArgs, SUB_Index indexArgs, SUB_ColorSensor colorSensorArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    canal = canalArgs;
    index = indexArgs;
    colorSensor = colorSensorArgs;
    addRequirements(canal,index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canal.setSpeedFront(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canal.setSpeedFront(0);
    colorSensor.popQ();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorSensor.isUnknown(colorSensor.peekQ()) || index.readBottomBanner();
  }
}
