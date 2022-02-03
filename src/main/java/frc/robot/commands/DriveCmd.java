

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem = new DriveSubsystem();
  public Supplier<Double> xSpeed,ySpeed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmd(DriveSubsystem subsystem, Supplier<Double> xSpeed, Supplier<Double> ySpeed) {
    
    m_subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Mecanum Drive Command started.");
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {

    // Drive
    m_subsystem.drive(ySpeed.get(),xSpeed.get());

    // Put numbers on SmartDashboard
    SmartDashboard.putNumber("ySpeed Value",ySpeed.get());
    SmartDashboard.putNumber("xSpeed Value",xSpeed.get());
  
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Mecanum Drive Command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
