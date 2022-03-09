// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_limeSpin extends CommandBase {
    Limelight limelight;
    Shooter shooter;
    boolean aimHigh;

  public CMD_limeSpin(Limelight limein, Shooter shootIn, boolean aimHigh) {
    this.limelight = limein;
    this.shooter = shootIn;
    this.aimHigh = aimHigh;
    addRequirements(limein, shootIn);
  }


  @Override
  public void initialize() {
    limelight.setLed(3);
  }


  @Override
  public void execute() {
    SmartDashboard.putNumber("TargetRPM", -(limelight.distRpm(limelight.getDistance(), aimHigh)));
    SmartDashboard.putBoolean("WithinRange?", !(limelight.getDistance() > Constants.MAX_RANGE));

    shooter.setRPM(limelight.distRpm(limelight.getDistance(), aimHigh));
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    limelight.setLed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((limelight.getTv() == false) || (limelight.getDistance() > Constants.MAX_RANGE)){
      return true;
    }else{
      return false;
    }
  }
}
