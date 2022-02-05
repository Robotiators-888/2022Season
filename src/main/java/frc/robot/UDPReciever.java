package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.function.Supplier;

import frc.robot.DataPacket;

public class UDPReciever<T extends DataPacket> extends Thread {
    final int port;
    DatagramSocket socket1;
    DatagramPacket dat1;
    byte[] receiveData1 = new byte[20];
    final Supplier<T> dataPacketConstructor;
    GenericBuffer buf;

    public UDPReciever(int port, Supplier<T> dataPacketConstructor, GenericBuffer<T> buf) {
        this.port = port;
        this.dataPacketConstructor = dataPacketConstructor;
        this.buf = buf;

        try {
            socket1 = new DatagramSocket(port);
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

                socket1.receive(dat1);
                buffer = dat1.getData();
                /*
                 * Testing UDP reciever code
                 * BufferData b = new BufferData(dat.getData());
                 * System.out.println(b.cycle + "" + b.x + "" + b.y);
                 * SmartDashboard.putNumber("Cycle", b.cycle);
                 */
                System.out.println("B2");

                buf.addData(dataPacketConstructor.get().fromBytes(dat1.getData()));
                System.out.println("B2");
                // dat.setData(null);

            } catch (IOException e) {

            }
        } while (true);
    }
}
