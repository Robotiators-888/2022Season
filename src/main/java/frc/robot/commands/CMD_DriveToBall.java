// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SUB_CameraData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_DriveToBall extends CommandBase {
    SUB_CameraData ballCam;
    Drivetrain drivetrain;

    double endPosError = 0.25;
    boolean camSelect = false; 
    double headingFinal = 0;
    Pose2d ballPos = new Pose2d();
    //false -> back 
    //true -> front


    public CMD_DriveToBall(SUB_CameraData cam, Drivetrain drive, boolean camSel) {
        this.ballCam = cam;
        this.camSelect = camSel;
        this.drivetrain = drive;
    }

    @Override
    public void initialize() {
        ballCam.setDirection(camSelect);
        ballPos = drivetrain.getPose().relativeTo(new Pose2d(Units.inchesToMeters(ballCam.getX()), Units.inchesToMeters(ballCam.getY()), drivetrain.getGyroHeading()));
        

        SmartDashboard.putNumber("ballPos x", ballPos.getX());
        SmartDashboard.putNumber("ballPos y", ballPos.getY());


    }

    //Ryansete controller TM
    @Override
    public void execute() {

        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
