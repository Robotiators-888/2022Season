// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Manages the canal subsystem, runs the front and back belts.
 */
public class SUB_CameraData extends SubsystemBase {

    public boolean direction = true; // true is forward, false is backward
    public static boolean forward = true;
    public static boolean backward = false;
    
  /** Creates a new CanalSubsystem. */
  public SUB_CameraData() {

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (direction == forward) {
      SmartDashboard.putString("direction", "front");
    } else {
      SmartDashboard.putString("direction", "back");
    }
  }

  public double getX(){
    if (direction){
      return SmartDashboard.getNumber("front_ball_x", 0);
    } else {
      return SmartDashboard.getNumber("back_ball_x", 0);
    }
  }

    public double getY(){
        if (direction){
        return SmartDashboard.getNumber("front_ball_y", 0);
        } else {
        return SmartDashboard.getNumber("back_ball_y", 0);
        }
    }

  public void toggleDirection() {
    direction = !direction;
  }

  public void setDirection(boolean direction) { // true is forward, false is backward
    this.direction = direction;
  }
}
