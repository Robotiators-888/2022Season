package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Limelight extends SubsystemBase {
  NetworkTable table;
  Boolean isHigh = true;

  public SUB_Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
  }

  public void periodic() {
    SmartDashboard.putNumber("dist", getDistance());
    SmartDashboard.putBoolean("High Shoot?", getHeight());

  }

  /**
   * Whether the limelight has any valid targets
   * 
   * @return boolean ture if target is found false if not
   */
  public boolean getTv() {
    if (table.getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Crosshair offset to target x-value
   * 
   * @return double of offset of target x-value
   */
  public double getTx() {
    return table.getEntry("tx").getDouble(0);
  }

  /**
   * Crosshair offset to target y-value
   * 
   * @return double of offset of target y-value
   */
  public double getTy() {
    return table.getEntry("ty").getDouble(0);
  }

  /**
   * Target area
   * 
   * @return double of target area
   */
  public double getTa() {
    return table.getEntry("ta").getDouble(0);
  }

  /**
   * Skew or rotation
   * 
   * @return double of skew/rotation
   */
  public double getTs() {
    return table.getEntry("ts").getDouble(0);
  }

  /**
   * Pipeline's latency contribition
   * 
   * @return double of pipeline latency
   */
  public double getTl() {
    return table.getEntry("tl").getDouble(0);
  }

  /**
   * Sets LED Mode
   */
  public void setLed(int value) {
    table.getEntry("ledMode").setNumber(value);

  }

  /**
   * Sets pipeline
   */
  public void setPipline(int value) {
    table.getEntry("pipeline").setNumber(value);

  }

  // Finds distance from robot to target and returns distance
  public double getDistance() {
    double h1 = 35.125;
    double h2 = 111;
    double a1 = 0.4625123;
    double a2 = Math.toRadians(this.getTy());

    return (double) ((h2 - h1) / (Math.tan(a1 + a2)));
  }

  public void toggleHeight(){
    if(isHigh){
      this.isHigh = false;
    }
    else{
      this.isHigh = true;
    }

  }

  public void setHeight(boolean input){
    this.isHigh = input;
  }

  public boolean getHeight(){
    return this.isHigh;
  }

  /**
   * calculates rpm for shooter to spin at given distance in inches from goal
   * @param dist false for rpm for low
   * @param shootHigh true for rpm for high goal
   * @return returns the rpm to hit specified goal
   */
  public int distRpm(double dist, boolean shootHigh) {

    if(shootHigh == true){
      return (int) ((24*dist) + 880);

    }
    else{

      return (int) (325 * (Math.sqrt(dist)));

    }
    
  } 

}
