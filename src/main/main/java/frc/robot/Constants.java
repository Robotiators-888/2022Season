// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drive Stuff
    public static final int FRONT_LEFT_ID = 21;
    public static final int REAR_LEFT_ID = 20;

    public static final int FRONT_RIGHT_ID = 23;
    public static final int REAR_RIGHT_ID = 22;

    // Joystick Stuff
    public static final int JOYSTICK_PORT = 0;

    // Joystick Control Stuff
    public static final int JOYSTICK_X_AXIS = 0;
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int JOYSTICK_Z_AXIS = 2;
    public static final double DEAD_ZONE = 0.3;
}
