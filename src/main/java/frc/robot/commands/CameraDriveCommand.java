// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;

public class CameraDriveCommand extends CommandBase {
  /** Creates a new CameraDrive. */
  Drivetrain drive;

    double ballX;
    double ballY;

  double moveXValue;
  double moveYValue;
  public CameraDriveCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

double XDeadzone = 3; // inches
double forwardDriveSpeed = 0.2;

  public void setBallLocation(double x, double y) {
    ballX = x;
    ballY = y;
  }

  @Override
  public void execute() {
    ballX = SmartDashboard.getNumber("front_ball_x", 0);
    ballY = SmartDashboard.getNumber("front_ball_y", 0);
    if (Math.abs(ballX) < XDeadzone) {
      // drive forward
        if (ballY>4){
            // drive forward at constant speed
            drive.setMotors(forwardDriveSpeed, forwardDriveSpeed);
        }
        else{
            // stop driving
            drive.setMotors(0,0);
        }
    }
    else{
        // turn to ball
        if(ballX > 3){
            moveXValue = 3;
          }else{
            moveXValue = ballX;
          }
          drive.setMotors((0.05)*(moveXValue),(-0.05)*(moveXValue));
    }
    
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