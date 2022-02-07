package frc.robot.UDP;

/**
 * Creates interface to recieve Data packets being sent
 */
public interface DataPacket {
    public abstract DataPacket fromBytes(byte[] bytes);
}
