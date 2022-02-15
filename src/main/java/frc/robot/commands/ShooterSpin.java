// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterSpin extends CommandBase {
  /** Creates a new ShooterSpin. */
  private Shooter shoot;
  private Joystick joystick;

  public ShooterSpin(Shooter subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = subsystem;
    this.joystick = joystick;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setRPM(-(int) (2888.5 * (1 - joystick.getRawAxis(3))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
