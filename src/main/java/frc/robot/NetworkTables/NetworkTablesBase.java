package frc.robot.NetworkTables;

import edu.wpi.first.networktables.EntryListenerFlags;
//imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Class
public class NetworkTablesBase {

  NetworkTableInstance inst;
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  TableEntryListener Ylistener;
  TableEntryListener Xlistener;
  NetworkTable piTable;

  public NetworkTablesBase() {
    run();
  }

  public void run() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    xEntry = table.getEntry("x");
    yEntry = table.getEntry("y");

    inst.startClientTeam(888); // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS 

    while (true) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }

      double x = SmartDashboard.getNumber("front_ball_x", 0);
      double y = SmartDashboard.getNumber("front_ball_y", 0);

 
      System.out.println("X: " + x + " Y: " + y);
    }
  }

}