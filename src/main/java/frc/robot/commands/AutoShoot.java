package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoShoot extends CommandBase {
 
  Limelight m_limelight;
  Shooter shoot;
  IndexSubsystem m_index;
  Drivetrain drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoShoot(Limelight limelight, IndexSubsystem index, Drivetrain drivetrain, Shooter shoot) {
    this.m_limelight = limelight;
    this.shoot = shoot;
    this.m_index = index;
    this.drive = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight, drivetrain, index, shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Limelight turned on and RPM displayed to dashboard
    m_limelight.setLed(0);
    SmartDashboard.putNumber("ShooterRPM", shoot.getRPM());
    SmartDashboard.putNumber("TargetRPM", -(m_limelight.distRpm(m_limelight.getDistance())));

    if((m_limelight.getTx() < 3) && (m_limelight.getTx() > -3)){

      // If limelight has valid target and its within 0-270 inches, fire shooter
      if ((m_limelight.getTv() == true) && (m_limelight.getDistance() > 0) && (m_limelight.getDistance() < 270)) {
        //If distance RPM > -5700, set shooter RPM to -5700
          if((-(m_limelight.distRpm(m_limelight.getDistance()))) < -5700){
            shoot.setRPM(-5700);
          }

          else{
          //Use distance to set RPM
            shoot.setRPM(-(m_limelight.distRpm(m_limelight.getDistance())));
          }
        

        // If the difference between the actual and target rpms is less than 150, start
        // index
        if ((double) Math.abs(shoot.getRPM() + m_limelight.distRpm(m_limelight.getDistance())) <= 250) {
          m_index.setSpeedTower(.7);
        }
      }

    }
    else{
      if((Math.abs(m_limelight.getTx()) <= 1)){

        drive.setMotors(0, 0);

      }
      if((Math.abs(m_limelight.getTx()) <= 5)){

        drive.setMotors(Math.signum(m_limelight.getTx())* 0.25, Math.signum(m_limelight.getTx())*-0.25);

      }
      else{

        drive.setMotors((0.03)*(m_limelight.getTx()),(-0.03)*(m_limelight.getTx()));

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
