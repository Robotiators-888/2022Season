package frc.robot.NetworkTables;

//imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Class
public class NetworkTablesBase{

    NetworkTableInstance inst;
    NetworkTableEntry xEntry;
    NetworkTableEntry yEntry;

    public NetworkTablesBase(){
        inst = NetworkTableInstance.create();
        inst.startServer("Pi Table", "10.8.88.86", 5802);
        
        NetworkTable piTable = inst.getTable("Pi Datatable");
        
        xEntry =piTable.getEntry("X");
        yEntry = piTable.getEntry("Y");


    }


}