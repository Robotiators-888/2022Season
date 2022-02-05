package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightDataPacket implements DataPacket {
    double x, y;
    int ballDetect;

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
