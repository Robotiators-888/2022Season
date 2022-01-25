// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommsBuffer {

	static ArrayList<BufferData> buf = new ArrayList<>(5);

	public static synchronized void addData(byte[] data) {
		buf.add(new BufferData(data));
	}

	public static synchronized Number[] getHighestCycle() {
		if(buf.isEmpty()) return null;
		BufferData b = buf.stream()
				.max((b1, b2) -> Integer.compare(b1.cycle, b2.cycle)).orElse(null);
		buf.clear();
		return new Number[] {b.cycle, b.x, b.y};
	}

}

class BufferData {

	public int cycle;
	public double x;
	public double y;

	public BufferData(byte[] data) {
		ByteBuffer bbuf = ByteBuffer.wrap(data);
		cycle = bbuf.getInt(0);
		x = bbuf.getDouble(4);
		y = bbuf.getDouble(12);

		SmartDashboard.putNumber("Cycle", cycle);
		SmartDashboard.putNumber("X Vals", x);
		SmartDashboard.putNumber("Y Vals", y);
	}

}
