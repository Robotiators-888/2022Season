package frc.robot.UDP;

import java.util.ArrayList;

/**
 * Class used for creating a Generic buffer to read through multiple packets
 */
public class GenericBuffer<T extends DataPacket> {
    ArrayList<DataPacket> buffer;

    /**
     * Creates array list for buffer
     */
    public GenericBuffer() {
        buffer = new ArrayList<>();
    }

    /**
     * Adds data to the buffer to be interpreted into Limelight data or Ball Tracker
     * data
     * 
     * @param packet
     */
    public synchronized void addData(DataPacket packet) {
        buffer.add(packet);
    }

    /**
     * 
     * @return Depending on the type either Limelight packet length or Ball Tracker
     *         data length
     */
    @SuppressWarnings("unchecked") synchronized T getData() {
        return (T) buffer.get(buffer.size() - 1);
    }
}
