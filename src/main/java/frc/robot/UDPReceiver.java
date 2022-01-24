package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPReceiver extends Thread {
	CommsBuffer CommunicationsBuffer = new CommsBuffer();
	DatagramSocket socket;
	DatagramPacket dat;
	byte[] receiveData = new byte[20];
	
	public void init() {
		try {
			socket = new DatagramSocket(5800);
			dat = new DatagramPacket(receiveData, receiveData.length);
		} catch (SocketException e) {
	
		}
	}
	
	@Override
	public void run() {
		
		
		do {
			try {
				socket.receive(dat);
				CommsBuffer.addData(dat.getData());
				dat.setData(null);
			} catch (IOException e) {
				
			}
		} while(true);
	}
}
