// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class CMD_ClimberToPosition extends CommandBase {

  SUB_Climber climber;
  private double destPos;
  
  /** Creates a new CMD_ClimberToPosition. */
  public CMD_ClimberToPosition(SUB_Climber climberArgs, double destPosArgs) {
    this.climber = climberArgs;
    this.destPos = destPosArgs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.lockSet(Value.kReverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(destPos-climber.getClimberPosition())>15){

      if (climber.getClimberPosition()<destPos){
        climber.speedSet(1);
      } else if (climber.getClimberPosition()>destPos){
        climber.speedSet(-1);
      }
      
    } else {
      if (climber.getClimberPosition()<destPos){
        climber.speedSet(0.5);
      } else if (climber.getClimberPosition()>destPos){
        climber.speedSet(-0.5);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.speedSet(0);
    climber.lockSet(Value.kForward); //lock the climber
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return(climber.getClimberPosition()>=destPos-Constants.CLIMBER_TOLERANCE && 
    climber.getClimberPosition()<=destPos+Constants.CLIMBER_TOLERANCE);

  }
}
