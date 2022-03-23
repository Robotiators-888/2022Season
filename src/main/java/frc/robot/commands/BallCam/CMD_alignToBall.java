// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallCam;

import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_CameraData;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_alignToBall extends CommandBase {
  SUB_CameraData cameraSub;
  SUB_Drivetrain drivetrain;
  boolean camSelect = false;
  // false -> back
  // true -> front

  double ballX;
  double ballY;

  double moveXValue;
  double moveYValue;

  // intake deadzone front is 6.5 inches
  static final double X_DEADZONE = 1; // inches

  static final double FORWARD_DRIVE_SPEED = 0.65;

  /**
   * turn robot to ball
   * 
   * @param cam
   * @param drive
   * @param camSel
   */
  public CMD_alignToBall(SUB_CameraData cam, SUB_Drivetrain drive, boolean camSel) {
    this.cameraSub = cam;
    this.camSelect = camSel;
    this.drivetrain = drive;
    addRequirements(cam, drive);
  }

  @Override
  public void initialize() {
    drivetrain.setIdleMode(IdleMode.kCoast);
    cameraSub.setDirection(camSelect);
  }

  // Ryansete controller TM
  // ok.
  @Override
  public void execute() {
    ballX = cameraSub.getX();

    if (cameraSub.ballDetected()) {
      // turn to ball
      if (Math.abs(ballX) > 3) {
        // if ball really far away, set turn speed to max
        moveXValue = 4;
        if (ballX < 0) {
          moveXValue = moveXValue * -1;
        } else {
          moveXValue = moveXValue * 1;
        }
      } else {
        // if less than 3 inches to right or left
        // then turn speed equals Xball inches
        moveXValue = ballX;
      }

      // multiply turn speeds down, then turn
      drivetrain.setMotors((0.06) * (moveXValue), (-0.06) * (moveXValue));
    } else {
      drivetrain.setMotors(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cameraSub.ballDetected() && (Math.abs(cameraSub.getX()) < X_DEADZONE)) {
      return true;
    } else {
      return false;
    }
  }

}
