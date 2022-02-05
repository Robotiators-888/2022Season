package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPReceiverLimelight extends Thread {
	byte[] buffer;
	CommsBufferLimelight CommunicationsBuffer = new CommsBufferLimelight();
	DatagramSocket socket;
	DatagramPacket dat;
	DatagramSocket socket1;
	DatagramPacket dat1;
	CommsBufferBall Com_buffer;
	byte[] receiveData = new byte[20];
	

	public void init() {
		try {
			System.out.println("1st INIT!!!");

			socket = new DatagramSocket(5801);
			dat = new DatagramPacket(receiveData, receiveData.length);
			/*
			socket1 = new DatagramSocket(5802);
			dat1 = new DatagramPacket(receiveData, receiveData.length);
			*/
		} catch (SocketException e) {

		}

	}

	@Override
	public void run() {
		
		// Receives data from the port 5801. Check CommsBuffer for what happens
		do {
			try {
				socket.receive(dat);
				/*
				 * Testing UDP reciever code
				 * BufferData b = new BufferData(dat.getData());
				 * System.out.println(b.cycle + "" + b.x + "" + b.y);
				 * SmartDashboard.putNumber("Cycle", b.cycle);
				 */
				System.out.println("A1");

				CommsBufferLimelight.addData(dat.getData());
				System.out.println("B1");
				/*
				socket1.receive(dat1);
				System.out.println("C");

				buffer = dat1.getData();
				Com_buffer = new CommsBufferBall(buffer);
				System.out.println("D");
				*/

			} catch (IOException e) {

			}
		} while (true);
	}
}
