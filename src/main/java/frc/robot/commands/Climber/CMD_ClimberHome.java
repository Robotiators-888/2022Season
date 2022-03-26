// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.SUB_Climber;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.*;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CMD_ClimberHome extends CommandBase {

  private SUB_Climber climber;
  private DoubleSolenoid cSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);
  /** Creates a new CMD_ClimberHome. */
  public CMD_ClimberHome(SUB_Climber climberArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climberArgs;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.speedSet(-0.85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.speedSet(0);
    cSolenoid.set(Value.kForward); //lock the climber
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getClimberPosition()<=0;
  }
}
