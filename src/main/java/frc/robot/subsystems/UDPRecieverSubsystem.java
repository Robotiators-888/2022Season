// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UDPReceiverBall;

public class UDPRecieverSubsystem extends SubsystemBase {
  /** Creates a new UDPReciever. */
  UDPReceiverBall receiverBall = new UDPReceiverBall();

  public UDPRecieverSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void createReceiver() {
    System.out.println("Thread 1 INIT");
    receiverBall.init();
    receiverBall.start();
    System.out.println("Thread 2 INIT");
  }

  public void runReceiver() {
   

  }

  public void BallTrackBuffer() {

  }

}
