package frc.robot.UDP;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class creates an reader for packets coming from the Rasp Pi and breaks it
 * down into
 * X coordinates(Float), Y coordinates(Float), Ball Detected(Int)
 * Looks for (Float, Float, Int)
 */
public class BallDataPacket implements DataPacket {
    static float x, y;
    int ballDetect;

    /**
     * Actually collects the data and formats and displays collected data
     */
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

    /**
     * Sends X coord to other parts of the robot from ball tracker
     * 
     * @return Float of X coordinate
     */
    public static float getX() {
        return x;
    }

    /**
     * Sends Y coord to other parts of the robot from ball tracker
     * 
     * @return Float of Y coordinate
     */
    public static float getY() {
        return y;
    }

}
