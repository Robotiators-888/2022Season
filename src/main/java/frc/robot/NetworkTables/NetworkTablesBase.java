package frc.robot.NetworkTables;

import edu.wpi.first.networktables.EntryListenerFlags;
//imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;

//Class
public class NetworkTablesBase{

    NetworkTableInstance inst;
    NetworkTableEntry xEntry;
    NetworkTableEntry yEntry;
    TableEntryListener Ylistener;
    TableEntryListener Xlistener;
    NetworkTable piTable;

    public NetworkTablesBase(){
        inst = NetworkTableInstance.getDefault();
        piTable = inst.getTable("Pi Datatable");
        inst.setServer("Pi Table Prime", 5802);
        
        piTable.addEntryListener("Pi Cam X", Xlistener, EntryListenerFlags.kNew);
        piTable.addEntryListener("Pi Cam Y", Ylistener, EntryListenerFlags.kNew);

    }

    public void networkTableInit(){
      //  Xlistener.valueChanged(piTable, "Pi Cam X", entry, value, flags);
    }


}