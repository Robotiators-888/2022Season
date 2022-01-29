// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UDPReceiverBall;
import frc.robot.UDPReceiverLimelight;
import java.nio.ByteBuffer;
import java.util.ArrayList;

public class UDPRecieverSubsystem extends SubsystemBase {
  /** Creates a new UDPReciever. */
  UDPReceiverLimelight receiverLimelight = new UDPReceiverLimelight();
  UDPReceiverBall receiverBall = new UDPReceiverBall();

  public UDPRecieverSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void createReceiver() {
    receiverLimelight.init();
    receiverBall.init();
  }

  public void runReceiver() {
    receiverLimelight.run();
    receiverBall.run();
  }

  public void BallTrackBuffer() {

  }

}
