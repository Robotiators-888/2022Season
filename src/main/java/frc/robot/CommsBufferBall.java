package frc.robot;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommsBufferBall {

    static ArrayList<BufferDataBall> buf = new ArrayList<>(32);

    public static synchronized void addDataBall(byte[] data) {
        buf.add(new BufferDataBall(data));
    }

    public static synchronized Number[] getByteArray() {

        return new Number[] {};
    }

}

class BufferDataBall {

    public int ballDetect;
    public float x;
    public float y;

    public BufferDataBall(byte[] data) {
        ByteBuffer bbuf = ByteBuffer.wrap(data);
        x = bbuf.getFloat(0);
        y = bbuf.getFloat(4);
        ballDetect = bbuf.getInt(8);

        SmartDashboard.putNumber("numBalls", ballDetect);
        SmartDashboard.putNumber("X coord", x);
        SmartDashboard.putNumber("Y coord", y);
    }
}