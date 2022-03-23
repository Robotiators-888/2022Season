// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_limeAlign extends CommandBase {
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  /**
   * turn to goal using limelight
   * @param limeIn
   * @param driveIn
   */
  public CMD_limeAlign(SUB_Limelight limeIn, SUB_Drivetrain driveIn) {
    this.limelight = limeIn;
    this.drivetrain = driveIn;
    addRequirements(limeIn, driveIn);
  }

  @Override
  public void initialize() {
    limelight.setLed(3);
  }

  @Override
  public void execute() {
    if ((Math.abs(limelight.getTx()) > 5)) {
      drivetrain.setMotors(Math.signum(limelight.getTx()) * 0.3, Math.signum(limelight.getTx()) * -0.3);

    } else {
      drivetrain.setMotors((0.03) * (limelight.getTx()), (-0.03) * (limelight.getTx()));

    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotors(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((limelight.getTv() && (limelight.getTx() < 4) && (limelight.getTx() > -4))) {
      return true;
    } else {
      return false;
    }
  }
}
