

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem = new DriveSubsystem();
  public Supplier<Double> LSpeed,RSpeed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmd(DriveSubsystem subsystem, Supplier<Double> LSpeed, Supplier<Double> RSpeed) {
    
    m_subsystem = subsystem;
    this.LSpeed = LSpeed;
    this.RSpeed = RSpeed;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drive Command started.");
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {

    // Drive
    m_subsystem.drive(LSpeed.get(),RSpeed.get());

    // Put numbers on SmartDashboard
    SmartDashboard.putNumber("Left Speed Value",LSpeed.get());
    SmartDashboard.putNumber("Right Speed Value",RSpeed.get());
  
  
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
