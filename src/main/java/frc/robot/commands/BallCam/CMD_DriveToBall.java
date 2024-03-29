// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallCam;

import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_CameraData;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;

public class CMD_DriveToBall extends CommandBase {
    SUB_CameraData cameraSub;
    SUB_Drivetrain drivetrain;

    boolean camSelect = false;
    // false -> back
    // true -> front

    /**
     * Drive robot straight to ball
     * 
     * @param cam
     * @param drive
     * @param camSel
     */

    double ballX;
    double ballY;

    double moveXValue;
    double moveYValue;
    // intake deadzone front is 6.5 inches
    static final double X_DEADZONE = 5.5; // inches
    // X deadzone back intake 3.5 inches
    static final double Y_DEADZONE = 4;
    static final double FORWARD_DRIVE_SPEED = 0.65;
    public boolean doneDriving = false;

    Timer driveTimeout = new Timer();

    public CMD_DriveToBall(SUB_CameraData cam, SUB_Drivetrain drive, boolean camSel) {
        this.cameraSub = cam;
        this.camSelect = camSel;
        this.drivetrain = drive;
        addRequirements(cam, drive);
    }

    @Override
    public void initialize() {
        drivetrain.setIdleMode(IdleMode.kCoast);
        cameraSub.setDirection(camSelect);
    }

    // Ryansete controller TM
    @Override
    public void execute() {
        if (cameraSub.ballDetected()) {
            if (cameraSub.direction == true) {
                drivetrain.setMotors(FORWARD_DRIVE_SPEED, FORWARD_DRIVE_SPEED);
            } else {
                drivetrain.setMotors(-FORWARD_DRIVE_SPEED, -FORWARD_DRIVE_SPEED);
            }
        } else {
            drivetrain.setMotors(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (cameraSub.ballDetected() && (cameraSub.getX() < 20)) {
            return true;
        } else {
            return false;
        }
    }

}
