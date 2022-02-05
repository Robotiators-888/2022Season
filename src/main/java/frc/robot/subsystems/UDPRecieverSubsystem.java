// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.UDPReciever;

import java.nio.ByteBuffer;
import java.util.ArrayList;

public class UDPRecieverSubsystem extends SubsystemBase {
  /** Creates a new UDPReciever. */

  public UDPRecieverSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void createReceiver() {
<<<<<<< HEAD
    System.out.println("Thread 1 INIT");
    receiverBall.init();
    receiverBall.start();
    System.out.println("Thread 2 INIT");
=======
>>>>>>> 8e65f6a (Generalized UDP Receiver Structure pt 2)
  }

  public void runReceiver() {

  }

  public void BallTrackBuffer() {

  }

}
