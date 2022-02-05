// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive Stuff
    public static final int FRONT_LEFT_ID = 12;
    public static final int REAR_LEFT_ID = 13;

    public static final int FRONT_RIGHT_ID = 10;
    public static final int REAR_RIGHT_ID = 11;

    // Joystick Stuff
    public static final int JOYSTICK_PORT = 0;

    // Joystick Control Stuff
    public static final int LEFT_AXIS = 1;
    public static final int RIGHT_AXIS = 5;
    public static final double DEAD_ZONE = 0.3;

    public static final int JOYSTICK_X_AXIS = 0;
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int JOYSTICK_Z_AXIS = 2;
    // color sensor stuff
    public static final double colorSensorId0 = 0;
    public static final double colorSensorId1 = 1;
    public static final double colorSensorId2 = 2;
    public static final double colorSensorId3 = 3;

    // Drivetrain Can IDs
    public static final int ID_LEFT_PRIMARY = 12;
    public static final int ID_LEFT_SECONDARY = 13;
    public static final int ID_RIGHT_PRIMARY = 10;
    public static final int ID_RIGHT_SECONDARY = 11;

    //---------------------------------- Auto Constants ---------------------------------- 

    public static final double TRACKWIDTH = 35;  // track width in inches
    public static final double WHEEL_RADIUS = 3; //wheel radius in inches
    public static final double GEARRATIO = 10.85; // number of rotations per one inch of movement
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            Units.inchesToMeters(TRACKWIDTH));

    // The Robot Characterization tool will help in obtaining
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/robot-characterization/index.html
    public static final double ksVolts = 0.25;
    public static final double kvVoltSecondsPerMeter = 0.28;
    public static final double kaVoltSecondsSquaredPerMeter = 0.03;
    public static final double kPDriveVel = 1.31;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}

