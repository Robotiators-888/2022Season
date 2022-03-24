// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.SUB_Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_zeroOdometry extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SUB_Drivetrain drive;

  public CMD_zeroOdometry(SUB_Drivetrain input) {
    this.drive = input;
    addRequirements(input);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.zeroEncoders();
    drive.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}