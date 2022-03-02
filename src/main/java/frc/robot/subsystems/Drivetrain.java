package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private boolean reversed = false;
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

  private Field2d field2d;

  DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getGyroHeading(),
      new Pose2d(0, 0, new Rotation2d()));

  public Drivetrain(Field2d input) {
    this.field2d = input;
    navx.setAngleAdjustment(0);

    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);
    leftPrimary.setInverted(true);
    leftSecondary.setInverted(true);

    setIdleMode(IdleMode.kCoast);
  }

  public void periodic() {
    
    driveOdometry.update(getGyroHeading(), this.rotationsToMeters(leftEncoder.getPosition()),
        this.rotationsToMeters(rightEncoder.getPosition()));

    field2d.setRobotPose(driveOdometry.getPoseMeters());

    // // smart dashboard logging
    SmartDashboard.putData("field", field2d);
    SmartDashboard.putNumber("x", driveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y", driveOdometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("rev L", leftEncoder.getPosition());
    // SmartDashboard.putNumber("rev R", rightEncoder.getPosition());
    SmartDashboard.putNumber("odoHead", driveOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("navHead", navx.getAngle());
    // SmartDashboard.putNumber("tester", this.rotationsToMeters(1));
    // SmartDashboard.putNumber("speed", getRate(leftEncoder.getVelocity()));
    SmartDashboard.putBoolean("Is Reversed", reversed);
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
   * Sets the reverse variable ito given value
   * @param state value to set reversed to
   */
  public void setReverse(boolean state){
    reversed = state;
  }

  /**
   * toggles state of reverse variable
   */
  public void toggleReverse(){
    if(reversed){
      reversed = false;
    }else {
      reversed = true;
    }
  }

  /**
   * @return boolean sotred in reverse variable
   */
  public boolean getReverse(){
    return reversed;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    groupLeft.setVoltage(leftVolts);
    groupRight.setVoltage(rightVolts);
    driveTrain.feed();
  }

  /**
   * sets what the motor does while idle
   * 
   * @param input the mode the moros should be put in (kBrake or kCoast)
   */
  public void setIdleMode(IdleMode input) {
    leftPrimary.setIdleMode(input);
    leftSecondary.setIdleMode(input);
    rightPrimary.setIdleMode(input);
    rightSecondary.setIdleMode(input);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getRate(leftEncoder.getVelocity()), getRate(rightEncoder.getVelocity()));
  }

  /**
   * zeros the encoder rotations
   */
  public void zeroEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * 
   * @param input encoder rotations from sparkmax
   * @return meters the robot has moved
   */
  public double rotationsToMeters(double input) {
    double wheelCirc = (2 * Math.PI * Constants.WHEEL_RADIUS);
    double rotationsPerInch = wheelCirc / Constants.GEARRATIO;
    return Units.inchesToMeters(rotationsPerInch * input);

  }

  /**
   * @param input rpm of drivetrain motor
   * @return returns rate of encoder in meters per second
   */
  public double getRate(double input) {
    return  (input / Constants.GEARRATIO) * ((2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_RADIUS)) / 60);
  }

  /**
   * 
   * @return rotation2d object with current heading
   */
  public Rotation2d getGyroHeading() {
    return new Rotation2d(Math.toRadians(-1 * navx.getAngle()));
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
    zeroHeading();
    navx.setAngleAdjustment(angle.getDegrees());
    zeroEncoders();
  }

  /**
   * Sets the robot's current pose to the given Pose2d.
   * 
   * @param position The position (both translation and rotation)
   */
  public void setPosition(Pose2d position) {
    zeroHeading();
    zeroEncoders();
    navx.setAngleAdjustment(position.getRotation().getDegrees());
    System.out.println(position.getX());
    System.out.println(position.getY());
    driveOdometry.resetPosition(position, position.getRotation());
    
  }

  /**
   * sets current heading to zero
   */
  public void zeroHeading() {
    navx.zeroYaw();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

}
