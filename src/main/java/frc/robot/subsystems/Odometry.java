// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getGyroHeading(),
            new Pose2d(5.0, 13.5, new Rotation2d()));
    AHRS navx = new AHRS();

    public Odometry() {
    }

    @Override
    public void periodic() {
        driveOdometry.update(getGyroHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    public Rotation2d getGyroHeading() {
        return new Rotation2d(Math.toRadians(navx.getYaw()));
    }

    /**
     * Sets the robot's current pose to the given x/y/angle.
     * 
     * @param x     The x coordinate
     * @param y     The y coordinate
     * @param angle The rotation component
     */
    public void setPosition(double x, double y, Rotation2d angle) {
        setPosition(new Pose2d(x, y, angle));
    }

    /**
     * Sets the robot's current pose to the given Pose2d.
     * 
     * @param position The position (both translation and rotation)
     */
    public void setPosition(Pose2d position) {
        driveOdometry.resetPosition(position, getGyroHeading());
    }


}
