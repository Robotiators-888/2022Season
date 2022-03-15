// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallCam;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SUB_CameraData;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_alignToBall extends CommandBase {
    SUB_CameraData ballCam;
    Drivetrain drivetrain;
    boolean camSelect = false;
    // false -> back
    // true -> front

    /**
     * turn robot to ball
     * 
     * @param cam
     * @param drive
     * @param camSel
     */
    public CMD_alignToBall(SUB_CameraData cam, Drivetrain drive, boolean camSel) {
        this.ballCam = cam;
        this.camSelect = camSel;
        this.drivetrain = drive;
        addRequirements(cam, drive);
    }

    @Override
    public void initialize() {
        ballCam.setDirection(camSelect);
    }

    // Ryansete controller TM
    @Override
    public void execute() {
        if (camSelect && ballCam.getX() > 0) {
            drivetrain.setMotors(0.25, -0.25);
        } else if (camSelect) {
            drivetrain.setMotors(-0.25, 0.25);
        } else if (!camSelect && ballCam.getX() > 0) {
            drivetrain.setMotors(-0.25, 0.25);
        } else if (!camSelect) {
            drivetrain.setMotors(0.25, -0.25);
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
        if ((ballCam.getX() < 1) && (ballCam.getX() > -1)) {
            return true;
        } else {
            return false;
        }
    }

}
