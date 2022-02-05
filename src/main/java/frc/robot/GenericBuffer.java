package frc.robot;

import java.util.ArrayList;

public class GenericBuffer<T extends DataPacket> {
    ArrayList<DataPacket> buffer;

    public GenericBuffer() {
        buffer = new ArrayList<>();
    }

    public synchronized void addData(DataPacket packet) {
        buffer.add(packet);
    }

    public synchronized T getData() {
        return (T) buffer.get(buffer.size() - 1);
    }
}
