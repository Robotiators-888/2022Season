package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallDataPacket implements DataPacket {
    float x, y;
    int ballDetect;

    @Override
    public DataPacket fromBytes(byte[] bytes) {
        ByteBuffer bbuf = ByteBuffer.wrap(bytes);
        x = bbuf.getFloat(0);
        y = bbuf.getFloat(4);
        ballDetect = bbuf.getInt(9);

        SmartDashboard.putNumber("numBalls", ballDetect);
        SmartDashboard.putNumber("X coord", x);
        SmartDashboard.putNumber("Y coord", y);
        System.out.println(x + " " + y + " " + ballDetect);
        return this;
    }
}
