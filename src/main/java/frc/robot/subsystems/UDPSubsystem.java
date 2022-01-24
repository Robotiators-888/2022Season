// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDPSubsystem extends SubsystemBase {
  /** Creates a new UDPSubsystem. 
   */

	DatagramSocket socket;
	DatagramPacket dat;
	byte[] receiveData = new byte[20];
	
	private void init() {
		try {
			socket = new DatagramSocket(888);
			dat = new DatagramPacket(receiveData, receiveData.length);
		} catch (SocketException e) {
	
		}
	}
	
	
	public void run() {
		init();
		
		do {
			try {
				socket.receive(dat);
			//	CommunicationsBuffer.addData(dat.getData());
				dat.setData(null);
			} catch (IOException e) {
				
			}
		} while(true);
	}
}
