// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    // Drivetrain Can IDs
    public static final int ID_LEFT_PRIMARY = 12;
    public static final int ID_LEFT_SECONDARY = 13;
    public static final int ID_RIGHT_PRIMARY = 10;
    public static final int ID_RIGHT_SECONDARY = 11;

    // Kinematics and odometry
    public static final double TRACKWIDTH = 30; // track width in inches
    public static final double WHEEL_RADIUS = 3;
    public static final double GEARRATIO = 8.58; // number of rotations per one inch of movement

    // Joystick
    // this tells which port of the driver station the joystick is in
    public static final int JOYSTICK_PORT = 0;

    // This tells us which part of the joystick will be used
    // 1 = to the left stick's y axis
    public static final int LEFT_AXIS = 1;
    // 4 = to the right stick's y axis
    public static final int RIGHT_AXIS = 5;

    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
}
