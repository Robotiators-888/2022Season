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
    // Drivetrain Can IDs
    public static final int ID_LEFT_PRIMARY = 20;
    public static final int ID_LEFT_SECONDARY = 21;
    public static final int ID_RIGHT_PRIMARY = 22;
    public static final int ID_RIGHT_SECONDARY = 23;

    // Climber Motor Can IDs
    public static final int CLIMBER_MOTOR_ID = 27;

    // Pneumatic Port IDs
    public static final int CLIMBER_LOCK_PORT_1 = 4;
    public static final int CLIMBER_LOCK_PORT_2 = 5;
    // Intake Can IDs
    public static final int MOTOR_ID = 13;
    public static final int FRONT_CANAL_ID = 10;
    public static final int TOWER_INDEX_ID = 26;
    public static final int BACK_CANAL_ID = 11;

    // Solenoid IDS
    public static final int solenoid_a = 0;
    public static final int solenoid_A = 1;

    public static final int solenoid_b = 2;
    public static final int solenoid_B = 3;

    // Joystick ports
    public static final int JOYSTICK_PORT = 0;
    public static final int LEFTJOYSTICK_PORT = 1;
    public static final int RIGHTSTICK_PORT = 2;

    // Joystick axis
    public static final int LEFT_AXIS = 1;
    public static final int RIGHT_AXIS = 5;
    public static final double DEAD_ZONE = 0.3;

    // PCM IDS
    public static final int PCM = 0;

    // Shooter rpms
    public static final int LOW_GOAL_RPMS = 3200;

    // ---------------------------------- Auto Constants
    // ----------------------------------

    // Auto Constants
    public static final double TRACKWIDTH = 35; // track width in inches
    public static final double WHEEL_RADIUS = 3; // wheel radius in inches
    public static final double GEARRATIO = 15.5; //gear ratio from output shaft of motor to wheel axle
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            Units.inchesToMeters(TRACKWIDTH));

    // The Robot Characterization tool will help in obtaining
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/robot-characterization/index.html
    public static final double ksVolts = 0.15431;
    public static final double kvVoltSecondsPerMeter = 1.9189;
    public static final double kaVoltSecondsSquaredPerMeter = 0.28843;
    public static final double kPDriveVel = 2.5217;

    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // UDP PORTS
    public static final int LIMELIGHT_PORT = 5801;
    public static final int BALL_PORT = 5802;

    public static final double TOWER_BELT_SPEED = 0.5;
    public static final int FLYWHEEL_MOTOR_ID = 24;
    public static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 25;

    public static final double BELT_SPEED = 0.75;
    public static final double ShooterSpeed = -0.15;

    public static final int DIO_PORT_0 = 9;
    public static final int DIO_PORT_1 = 8;
    
    public static final double P_VALUE = 0.00015;
    public static final double I_VALUE = 0;
    public static final double D_VALUE = 0;
    public static final double F_VALUE = 0.000183;

    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 60;
    
    public static final double MAX_RANGE = 250;

    public static final int FRONT_COLOR_SENSOR_ID = 0;
    public static final int BACK_COLOR_SENSOR_ID = 1;

}
