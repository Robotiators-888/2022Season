package frc.robot.UDP;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.function.Supplier;

/**
 * Class that creates datagram packets and sockets. Collects data pckets from
 * the ports it's checking and sends it to the other files
 */
public class UDPReciever<T extends DataPacket> extends Thread {
    final int port;
    DatagramSocket socket1;
    DatagramPacket dat1;
    byte[] receiveData1 = new byte[20];
    final Supplier<T> dataPacketConstructor;
    GenericBuffer<T> buf;

    /**
     * Creates the ojects required to recieve data
     * 
     * @param port                  Port that packets are being sent to either 5801
     *                              or 5802
     * @param dataPacketConstructor constructs a body for data packet
     * @param buf                   Creates data buffer
     */
    public UDPReciever(int port, Supplier<T> dataPacketConstructor, GenericBuffer<T> buf) {
        this.port = port;
        this.dataPacketConstructor = dataPacketConstructor;
        this.buf = buf;

        try {
            socket1 = new DatagramSocket(port);
            dat1 = new DatagramPacket(receiveData1, receiveData1.length);
        } catch (SocketException e) {}
    }

    /**
     * Runs Reciever listening for both Limelight packets and Ball tracker packets
     */
    @Override
    public void run() {
        // Receives data from the port 8888. Check CommsBuffer for what happens
        do {
            try {
                socket1.receive(dat1);
                buf.addData(dataPacketConstructor.get().fromBytes(dat1.getData()));
            } catch (IOException e) {}
        } while (true);
    }
}
