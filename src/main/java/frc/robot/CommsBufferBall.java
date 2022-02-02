package frc.robot;

import java.nio.ByteBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommsBufferBall {
/*
    static ArrayList<BufferDataBall> buf = new ArrayList<>(32);

    public static synchronized void addDataBall(byte[] data) {
        buf.add(new BufferDataBall(data));
       
    }
    /*
    public static synchronized Number[] getByteArray() {
        if (buf.isEmpty())
        return null;
    BufferDataBall b = buf.stream().
    b.getfloat();
    buf.clear();
    
        return new Number[] {};
    }
    */

    public int ballDetect=0;
    public float x=0;
    public float y=0;

    public CommsBufferBall(byte data[]) {
        ByteBuffer bbuf = ByteBuffer.wrap(data);
        x = bbuf.getFloat(0);
        y = bbuf.getFloat(4);
        ballDetect = bbuf.getInt(9);

        SmartDashboard.putNumber("numBalls", ballDetect);
        SmartDashboard.putNumber("X coord", x);
        SmartDashboard.putNumber("Y coord", y);
        System.out.println(x + " " + y + " " + ballDetect);
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