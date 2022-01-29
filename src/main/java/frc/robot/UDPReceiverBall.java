package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPReceiverBall extends Thread {
    CommsBufferBall CommunicationsBuffer = new CommsBufferBall();
    DatagramSocket socket;
    DatagramPacket dat;
    byte[] receiveData = new byte[20];

    public void init() {
        try {
            socket = new DatagramSocket(8888);
            dat = new DatagramPacket(receiveData, receiveData.length);
        } catch (SocketException e) {

        }
    }

    @Override
    public void run() {

        // Receives data from the port 8888. Check CommsBuffer for what happens
        do {
            try {
                socket.receive(dat);
                /*
                 * Testing UDP reciever code
                 * BufferData b = new BufferData(dat.getData());
                 * System.out.println(b.cycle + "" + b.x + "" + b.y);
                 * SmartDashboard.putNumber("Cycle", b.cycle);
                 */
                CommsBufferBall.addDataBall(dat.getData());
                // dat.setData(null);

            } catch (IOException e) {

            }
        } while (true);
    }
}
