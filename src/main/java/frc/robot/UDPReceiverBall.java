package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPReceiverBall extends Thread {
   // CommsBufferBall CommunicationsBuffer = new CommsBufferBall();
    DatagramSocket socket1;
    DatagramPacket dat1;
    byte[] receiveData1 = new byte[20];

    public void init() {
        try {
            System.out.println("2nd INIT!!!");
            socket1 = new DatagramSocket(5802);
            dat1 = new DatagramPacket(receiveData1, receiveData1.length);
        } catch (SocketException e) {

        }
    }

    @Override
    public void run() {
        byte[] buffer = new byte[20];
        // Receives data from the port 8888. Check CommsBuffer for what happens
        do {
            try {
                System.out.println("A2");

                socket1.receive(dat1);
                buffer = dat1.getData();
                /*
                 * Testing UDP reciever code
                 * BufferData b = new BufferData(dat.getData());
                 * System.out.println(b.cycle + "" + b.x + "" + b.y);
                 * SmartDashboard.putNumber("Cycle", b.cycle);
                 */
                System.out.println("B2");

                CommsBufferBall b = new CommsBufferBall(buffer);
                System.out.println("B2");
                // dat.setData(null);

            } catch (IOException e) {

            }
        } while (true);
    }
}
