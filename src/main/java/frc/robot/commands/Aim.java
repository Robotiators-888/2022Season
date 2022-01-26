package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Aim extends CommandBase{
    Limelight m_limelight;
    Drivetrain drive;

    public Aim(Limelight subsystem, Drivetrain drive) {
        this.m_limelight = subsystem;
        this.drive = drive;
    
    
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, drive);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_limelight.setLed(0);
    
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
      m_limelight.setLed(0);
      drive.setMotors((0.04)*(m_limelight.getTx()),(0.04)*(m_limelight.getTx()));
      SmartDashboard.putNumber("Speed", (0.04)*(m_limelight.getTx()));
      }
         
       
      
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_limelight.setLed(1);
        drive.setMotors(0, 0);
        
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
          
       // return m_limelight.getTx() <0.3 && m_limelight.getTx() > -0.3;
      
       
      }
}
