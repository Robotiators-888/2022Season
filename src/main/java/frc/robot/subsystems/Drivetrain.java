package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  // create motor controller objects
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.ID_LEFT_PRIMARY,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.ID_LEFT_SECONDARY,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.ID_RIGHT_PRIMARY,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSecondary = new CANSparkMax(Constants.ID_RIGHT_SECONDARY,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  // create a speed controller group for each side
  private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
  private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

  // create a drive train group with the speed controller groups
  private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);

  private RelativeEncoder leftEncoder = leftPrimary.getEncoder();
  private RelativeEncoder rightEncoder = rightPrimary.getEncoder();

  AHRS navx = new AHRS(SerialPort.Port.kMXP);

  DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getGyroHeading(),
      new Pose2d(0, 0, new Rotation2d()));

  private final Field2d field2d = new Field2d();

  // kinematics
  // private DifferentialDriveKinematics kinematics = new
  // DifferentialDriveKinematics(Constants.TRACKWIDTH * 0.0254);

  private double leftEncoderTare = 0;
  private double rightEncoderTare = 0;

  public Drivetrain() {
  }

  public void periodic() {
    driveOdometry.update(getGyroHeading(), this.rotationsToMeters(this.getEncoderLeft()),
        this.rotationsToMeters(this.getEncoderRight()));

    field2d.setRobotPose(driveOdometry.getPoseMeters());
    // smart dashboard logging
    SmartDashboard.putData("field", field2d);
    SmartDashboard.putNumber("x", driveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y", driveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("rev L", this.getEncoderLeft());
    SmartDashboard.putNumber("rev R", this.getEncoderRight());
    SmartDashboard.putNumber("odoHead", driveOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("navHead", navx.getYaw());
    SmartDashboard.putNumber("tester", this.rotationsToMeters(1));
  }

  /**
   * Sets the drivetrain to inverted
   * 
   * @param invert the invert state of the drivetrain. True is inverted, False is
   *               not inverted.
   */
  public void invertDrive(boolean invert) {
    leftPrimary.setInverted(invert);
    rightPrimary.setInverted(invert);
    leftSecondary.setInverted(invert);
    rightSecondary.setInverted(invert);
  }

  /**
   * Sets speed of the motors in the drivetrain
   * 
   * @param leftSpeed  Speed of the left drivetrain
   * @param rightSpeed Speed of right drivetrain
   * @param Speed      set a precentage of max speed the robot can use, if not
   *                   provided will default to full power
   */
  public void setMotors(double leftSpeed, double rightSpeed, double Speed) {
    driveTrain.tankDrive(leftSpeed * Speed, rightSpeed * Speed);
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * 
   * @return returns the number of rotations of the left motor since last reset
   */
  public double getEncoderLeft() {
    return (leftEncoder.getPosition() - leftEncoderTare);
  }

  /**
   * 
   * @return returns the number of rotations of the right motor since last reset
   */
  public double getEncoderRight() {
    return -1 * (rightEncoder.getPosition() - rightEncoderTare);
  }

  /**
   * zeros the encoder value for the left side
   */
  public void zeroLeft() {
    leftEncoderTare = leftEncoder.getPosition();
  }

  /**
   * zeros the encoder value for the left side
   */
  public void zeroRight() {
    rightEncoderTare = rightEncoder.getPosition();
  }

  /**
   * 
   * @param input encoder rotations from sparkmax
   * @return meters the robot has moved
   */
  public double rotationsToMeters(double input) {
    double wheelCirc = (2 * 3.14 * Constants.WHEEL_RADIUS);
    double rotationsPerInch = wheelCirc / Constants.GEARRATIO;
    return 0.0254 * (rotationsPerInch * input);

  }

  /**
   * 
   * @return rotation2d object with current heading
   */
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

  /**
   * sets current heading to zero
   */
  public void zeroHeading() {
    navx.zeroYaw();

  }

}
