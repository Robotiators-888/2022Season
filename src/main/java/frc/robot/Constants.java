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
    //Drivetrain Can IDs
    public static final int ID_LEFT_PRIMARY = 20;  
    public static final int ID_LEFT_SECONDARY = 21;  
    public static final int ID_RIGHT_PRIMARY = 22;  
    public static final int ID_RIGHT_SECONDARY = 23;  

    //Kinematics and odometry
    public static final double TRACKWIDTH = 30;  //track width in inches
    public static final double ROTATIONS_PER_INCH = 0.5; //number of rotations per one inch of movement 

}
