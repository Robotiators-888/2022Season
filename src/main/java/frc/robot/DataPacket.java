package frc.robot;

/**
 * Creates interface to recieve Data packets being sent
 */
public interface DataPacket {
    public abstract DataPacket fromBytes(byte[] bytes);
}
