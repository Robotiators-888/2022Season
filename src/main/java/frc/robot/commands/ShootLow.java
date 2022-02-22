package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootLow extends CommandBase {
  Limelight m_limelight;
  Shooter shoot;
  IndexSubsystem m_index;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootLow(Limelight subsystem, Shooter shoot, IndexSubsystem m_index) {
    this.m_limelight = subsystem;
    this.shoot = shoot;
    this.m_index = m_index;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight, m_index, shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setLed(0);

    // If limelight has valid target and its within 50-270 inches, fire shooter
    if ((m_limelight.getTv() == true) && (m_limelight.getDistance() > 0) && (m_limelight.getDistance() < 270)) {
      shoot.setRPM(-(shoot.lowDistRpm(m_limelight.getDistance())));
      
      // If the difference between the actual and target rpms is less than 150, start
      // index
      if ((double) Math.abs(shoot.getRPM() + shoot.lowDistRpm(m_limelight.getDistance())) <= 400) {
        m_index.setSpeedTower(.5);
        // If requirements arent met at any time, set index and turret to 0
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLed(1);
    m_index.setSpeedTower(0);
    shoot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
