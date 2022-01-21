// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Limelight m_limelight = new Limelight();
  private Shooter shoot = new Shooter();
  XboxController controller = new XboxController(0);
  private Index m_index = new Index();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
//Displays Limelight and RPM data on dashboard
    SmartDashboard.putNumber("tx", m_limelight.getTx());
    SmartDashboard.putNumber("ty", m_limelight.getTy());
    SmartDashboard.putNumber("ta", m_limelight.getTa());
    SmartDashboard.putNumber("ts", m_limelight.getTs());
    SmartDashboard.putNumber("tl", m_limelight.getTl());
    SmartDashboard.putNumber("Distance", m_limelight.getDistance());
    SmartDashboard.putNumber("Actual Rpm", shoot.getRPM());
    SmartDashboard.putNumber("SetRPM", shoot.distRpm(m_limelight.getDistance()));

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
//Sets PID values and turns Limelight off
    shoot.setPIDF(0.0004, 0.0, 0.0, 0.000288);
    m_limelight.setLed(1);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
// When controller A button is pressed, turn on LimelightLED and run following if statements
    if (controller.getAButton()) {
      m_limelight.setLed(0);
      SmartDashboard.putBoolean("Target Accquired", m_limelight.getTv());
//If limelight has valid target and its within 50-200 inches, fire shooter
      if ((m_limelight.getTv() == true) && (m_limelight.getDistance() > 50) && (m_limelight.getDistance() < 230)) {
        shoot.setRPM(shoot.distRpm(m_limelight.getDistance()));
//If the difference between the actual and target rpms is less than 150, start index
        if ((double) Math.abs(shoot.getRPM() - shoot.distRpm(m_limelight.getDistance())) <= 150) {
          m_index.setSpeed(-0.5);
//If requirements arent met at any time, set index and turret to 0
        } else {
          m_index.setSpeed(0);
          shoot.setSpeed(0);
        }

      } else {
        m_index.setSpeed(0);
        shoot.setSpeed(0);
      }
    }else{
    shoot.setSpeed(0);
    m_index.setSpeed(0);
    m_limelight.setLed(1);
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
