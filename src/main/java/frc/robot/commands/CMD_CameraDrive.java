 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SUB_CameraData;

import java.lang.Math;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.CameraDriveSubsystem;

public class CMD_CameraDrive extends CommandBase {
  /** Creates a new CameraDrive. */
  Drivetrain drive;
  Timer driveTimeout = new Timer();
  SUB_CameraData cameraSub;

  double ballX;
  double ballY;

  double moveXValue;
  double moveYValue;
  // intake deadzone front is 6.5 inches
  static final double X_DEADZONE = 5.5; // inches
  //X deadzone back intake 3.5 inches
  static final double Y_DEADZONE = 4;
  static final double FORWARD_DRIVE_SPEED = 0.40;
  public boolean doneDriving = false;
  
  // driving to Y:50 x:25 inches

  public CMD_CameraDrive(Drivetrain drivetrain, SUB_CameraData cameraData) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    this.cameraSub = cameraData;

    drivetrain.setIdleMode(IdleMode.kCoast);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    ballX = cameraSub.getX();
    ballY = cameraSub.getY();

    
      // drive forward 
      
        // drive forward at constant speed

        /// ball close keep moving

        driveTimeout.start();
        if (!(driveTimeout.hasElapsed(0.7))) {
          // drive forward or backward
          if (cameraSub.direction == CameraDriveSubsystem.forward) {
            drive.setMotors(FORWARD_DRIVE_SPEED, FORWARD_DRIVE_SPEED);   
          } else {
            drive.setMotors(-FORWARD_DRIVE_SPEED, -FORWARD_DRIVE_SPEED);
          }
          doneDriving = false;  // keep driving
        } else {
          if (ballY > 40) { // reset timer only if we can see the ball
            driveTimeout.stop();
            driveTimeout.reset();
          }
          else{
            drive.setMotors(0, 0);
            doneDriving = true;
          }
        }

        // drive.setMotors(0,0);

      

      if (Math.abs(ballX) < X_DEADZONE) {
        // in deadzone dont turn
    } else {
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
      drive.setMotors((0.06) * (moveXValue), (-0.06) * (moveXValue));

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (doneDriving) {
      return true;
    } else {
      return false;
    }
  }

}