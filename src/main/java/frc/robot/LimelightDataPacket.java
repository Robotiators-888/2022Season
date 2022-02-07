package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class creates an reader for packets coming from the Limelight and breaks it
 * down into
 * X coordinates(Double), Y coordinates(Double)
 *
 */
public class LimelightDataPacket implements DataPacket {
    double x, y;
    int ballDetect;

    /**
     * Actually collects the data and formats and displays collected data
     */
    @Override
    public DataPacket fromBytes(byte[] bytes) {
        ByteBuffer bbuf = ByteBuffer.wrap(bytes);
        x = bbuf.getDouble(0);
        y = bbuf.getDouble(9);

        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);

        return this;
    }
}
