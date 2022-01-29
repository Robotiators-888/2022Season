// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FMSSubsystem extends SubsystemBase {

  DriverStation station;

  /** Creates a new FMSSubsystem. */
  public FMSSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void findInfo(Alliance alliance, MatchType matchtype, String EventName, String GameSpecMsg, int matchNumber,
      int allianceLocation, double matchTime) {
    alliance = DriverStation.getAlliance();
    matchTime = DriverStation.getMatchTime();
    EventName = DriverStation.getEventName();
    matchNumber = DriverStation.getMatchNumber();
    allianceLocation = DriverStation.getLocation();
    matchtype = DriverStation.getMatchType();
    GameSpecMsg = DriverStation.getGameSpecificMessage();

    if (alliance == Alliance.Blue) {
      SmartDashboard.putString("Alliance Color", "Blue");
    } else if (alliance == Alliance.Red) {
      SmartDashboard.putString("Alliance Color", "Red");
    } else {
      SmartDashboard.putString("Alliance Color", "Invalid");
    }

    SmartDashboard.putNumber("Alliance Location", allianceLocation);
    SmartDashboard.putString("Game Specific Message", GameSpecMsg);
    SmartDashboard.putString("Event Name", EventName);
    SmartDashboard.putNumber("Match Number", matchNumber);
    SmartDashboard.putNumber("Match Time", matchTime);

  }

}
