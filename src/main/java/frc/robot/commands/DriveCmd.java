/* 
* Kevin Hwang
* Lasted edited on 10/26/2021
* Calls the subsystem to make robot move using Mecanum wheels.
*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MecanumDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  public Supplier<Double> xSpeed,ySpeed,zRotation;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MecanumDriveCommand(DriveSubsystem subsystem,Supplier<Double> xSpeed, Supplier<Double> ySpeed,Supplier<Double> zRotation) {
    
    m_subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
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
    m_subsystem.drive(ySpeed.get(),xSpeed.get(),zRotation.get());

    // Put numbers on SmartDashboard
    SmartDashboard.putNumber("ySpeed Value",ySpeed.get());
    SmartDashboard.putNumber("xSpeed Value",xSpeed.get());
    SmartDashboard.putNumber("zRotation Value",zRotation.get());
  
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
