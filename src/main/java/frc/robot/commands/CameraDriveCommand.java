// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;

public class CameraDriveCommand extends CommandBase {
  /** Creates a new CameraDrive. */
  Drivetrain drive;
  Timer diveTimeout = new Timer();

  double ballX;
  double ballY;

  double moveXValue;
  double moveYValue;
  static final double X_DEADZONE = 6.5; // inches
  static final double Y_DEADZONE = 4;
  static final double FORWARD_DRIVE_SPEED = 0.6;

  public CameraDriveCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    ballX = SmartDashboard.getNumber("front_ball_x", 0);
    ballY = SmartDashboard.getNumber("front_ball_y", 0);
    if (Math.abs(ballX) < X_DEADZONE) {
      // drive forward
      if (ballY > 4) {
        // drive forward at constant speed

        /// ball close keep moving

        diveTimeout.start();
        if (!(diveTimeout.hasElapsed(0.5))) {
          drive.setMotors(FORWARD_DRIVE_SPEED, FORWARD_DRIVE_SPEED);
        } else {
          drive.setMotors(0, 0);
          diveTimeout.stop();
          diveTimeout.reset();
        }

        // drive.setMotors(0,0);

      }
    } else {
      // turn to ball
      if (Math.abs(ballX) > 3) {
        moveXValue = 4;
        if (ballX < 0) {
          moveXValue = moveXValue * -1;
        } else {
          moveXValue = moveXValue * 1;
        }
      } else {
        moveXValue = ballX;
      }
      drive.setMotors((0.07) * (moveXValue), (-0.07) * (moveXValue));

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
