// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallCam;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SUB_CameraData;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

  
 


public class CMD_alignToBall extends CommandBase {
  SUB_CameraData cameraSub;
  Drivetrain drivetrain;
  boolean camSelect = false;
  // false -> back
  // true -> front

// TODO: write a class to handle adding degrees and > < operators done. 
// TODO: for the deadzone check if the gyro is within a certain range of ball angle done.
// TODO: properly convert gryo cords from world to relative cords done.

  double ballX;
  double ballY;
  double ballAngle;
  double gyroHeading;
  double initGyroHeading;

  double moveXValue;
  double moveYValue;

  // intake deadzone front is 6.5 inches
  //static final double X_DEADZONE = 1; //  inches
  // 10 was close enough to 90 degrees to be on target
  static final double angleDeadzone = 10; // degrees


  static double DRIVE_SPEED = 0.50;

  public double degreeDistance(double degree1, double degree2){
    double p1 = Math.abs(degree1 - degree2);
    double p2 = 360 - p1;

    return Math.min(p1,p2);
  }

  public double angleWrap(double degree){
    if (degree > 359){ // 360 = 0! ex: 360 - 360 = 0
      return degree - 360;
    }
    else if (degree < 0){
      return degree + 360;
    }
    else{
      return degree;
    }
  }

  public boolean isLeftRight(double target,double gyro) {
    int distance = (int)(this.degreeDistance(target, gyro));

    int gyroMinusDist = (int)(gyro-distance);
    int gyroPlusDist = (int)(gyro+distance);

    int intTarget = (int)target;
    int intGyro = (int)gyro;


    System.out.println("gyro: " + gyro + " target: " + target + " distance: " + distance+ " gyroMinusDist: " + gyroMinusDist + " gyroPlusDist: " + gyroPlusDist+ "leftOrRight"+((gyroPlusDist==intTarget)?true:false));

    if (gyroPlusDist == intTarget){
      return true;
    }
    else if (gyroMinusDist == intTarget){
      return false;
    }
    else{
      System.out.println("gyro heading not working!!");
    }

    return false;
    //throw new  Exception("CMD_alignToBall.java: gyro:" + gyro + " target:" + target + " distance:" + distance + " gyroMinusDist:" + gyroMinusDist + " gyroPlusDist:" + gyroPlusDist + " intTarget:" + intTarget + " intGyro:" + intGyro);


  }

  public boolean inDeadZone(){
    double distance = this.degreeDistance(this.ballAngle, getCurrentGyroRelative());

    if (distance < angleDeadzone){
      return true;
    }
    else{
      return false;
    }
  }


  public double getCurrentGyroRelative(){
    return angleWrap(this.gyroHeading - this.initGyroHeading);
  }
  /**
   * turn robot to ball
   * 
   * @param cam
   * @param drive
   * @param camSel
   */
  public CMD_alignToBall(SUB_CameraData cam, Drivetrain drive, boolean camSel) {
    this.cameraSub = cam;
    this.camSelect = camSel;
    this.drivetrain = drive;
    addRequirements(cam, drive);
  }

  @Override
  public void initialize() {
    drivetrain.setIdleMode(IdleMode.kCoast);
    cameraSub.setDirection(camSelect);
    // test angle to see if its working
    ballAngle = cameraSub.getAngle();
    DRIVE_SPEED = 0.50;
    initGyroHeading = drivetrain.getGyroHeading().getDegrees();

    System.out.println("init gyro: " + initGyroHeading+ " ball angle: " + ballAngle + " relative Gyro"+getCurrentGyroRelative());
  }

  // Ryansete controller TM
  // ok.
  @Override
  public void execute() {
    // if our ball angle is 0 keep waiting for a non zero angle
    // if (this.ballAngle == 0){
    //   System.out.print("ball angle is 0, waiting for non zero angle");
    //   this.ballAngle = cameraSub.getAngle();
    // }
    // else{
        SmartDashboard.putNumber("Yaw Rel", getCurrentGyroRelative());

      double DRIVE_MULTIPLIER = 0.35; // this is our effective speed

        this.gyroHeading = drivetrain.getGyroHeading().getDegrees();

        if(inDeadZone()){
          DRIVE_MULTIPLIER = 0;
        }else if(isLeftRight(ballAngle, getCurrentGyroRelative()) == true){
          DRIVE_MULTIPLIER *= 1;
        }else if(isLeftRight(ballAngle, getCurrentGyroRelative()) == false){
          DRIVE_MULTIPLIER *= -1;
        }

        drivetrain.setMotors((1*DRIVE_MULTIPLIER) ,(-1*DRIVE_MULTIPLIER));
    //}
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    this.gyroHeading = drivetrain.getGyroHeading().getDegrees();
    // if (inDeadZone()) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }

}
