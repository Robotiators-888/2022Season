// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_curvatureDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SUB_Drivetrain drive;
  private Supplier<Double> Left, Right;
  private Supplier<Boolean> canTurnStationary;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CMD_curvatureDrive(SUB_Drivetrain drivetrain, Supplier<Double> L, Supplier<Double> R, Supplier<Boolean> canTurn) {
    this.drive = drivetrain;
    this.Left = L;
    this.Right = R;
    this.canTurnStationary = canTurn;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setIdleMode(IdleMode.kCoast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drive.setMotorsCurvature(Left.get()*-1, Right.get()*-1, canTurnStationary); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotorsArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
