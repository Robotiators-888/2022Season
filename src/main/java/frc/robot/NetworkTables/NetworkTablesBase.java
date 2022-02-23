package frc.robot.NetworkTables;

import edu.wpi.first.networktables.EntryListenerFlags;
//imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;

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

  public static void run() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    NetworkTableEntry xEntry = table.getEntry("x");
    NetworkTableEntry yEntry = table.getEntry("y");

    inst.startClientTeam(888); // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS

    while (true) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }

      double x = xEntry.getDouble(0.0);
      double y = yEntry.getDouble(0.0);
      System.out.println("X: " + x + " Y: " + y);
    }
  }

}