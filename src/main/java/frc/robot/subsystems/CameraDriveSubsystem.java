// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *  Allows getting location of ball based on camera from pi
 *  Allows switching between cameras for streaming and ballDetection
 */
public class CameraDriveSubsystem extends SubsystemBase {

    public boolean direction = true; // true is forward, false is backward
    public static final boolean forward = true;
    public static final boolean backward = false;
  /** Creates a new CanalSubsystem. */
  public CameraDriveSubsystem() {

  }
  
  /**
 * sets active camera front/back to smartdashboard
 * so PIs can switch there camera
 */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (direction == forward) {
      SmartDashboard.putString("direction", "front");
    } else {
      SmartDashboard.putString("direction", "back");
    }
  }

   /**
 * get the inches left or right from the ball from pi cam
 */
  public double getX(){
    if (direction){
      return SmartDashboard.getNumber("front_ball_x", 0);
    } else {
      return SmartDashboard.getNumber("back_ball_x", 0);
    }
  }

   /**
 * get the inches forward to the ball from pi cam
 */
    public double getY(){
        if (direction){
        return SmartDashboard.getNumber("front_ball_y", 0);
        } else {
        return SmartDashboard.getNumber("back_ball_y", 0);
        }
    }


   /**
 * toggle between front or back cam for streaming and ballDetection
 */
  public void toggleDirection() {
    direction = !direction;
  }
}
