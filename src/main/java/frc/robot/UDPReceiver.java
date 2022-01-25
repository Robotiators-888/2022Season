package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UDPReceiver extends Thread {
	CommsBuffer CommunicationsBuffer = new CommsBuffer();
	DatagramSocket socket;
	DatagramPacket dat;
	byte[] receiveData = new byte[20];
	
	public void init() {
		try {
			socket = new DatagramSocket(5801);
			dat = new DatagramPacket(receiveData, receiveData.length);
		} catch (SocketException e) {
	
		}
	}
	
	@Override
	public void run() {
		
		
		do {
			try {
				socket.receive(dat);
				/*
				Testing UDP reciever code
				BufferData b = new BufferData(dat.getData());
				System.out.println(b.cycle + "" + b.x + "" + b.y);
				SmartDashboard.putNumber("Cycle", b.cycle);
				*/
				CommsBuffer.addData(dat.getData());
				//dat.setData(null);
                
			} catch (IOException e) {
				
			}
		} while(true);
	}
}
