// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<< HEAD
import frc.robot.subsystems.IndexSubsystem;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

=======
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleopIndexTester.java
=======
import frc.robot.subsystems.IndexSubsystem;


>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9:src/main/java/frc/robot/commands/indexRun.java
import edu.wpi.first.wpilibj2.command.CommandBase;

<<<<<<< HEAD:src/main/java/frc/robot/commands/teleopIndexTester.java
import frc.robot.subsystems.IndexSubsystem;

public class teleopIndexTester extends CommandBase {

  private IndexSubsystem index;
  /** Creates a new teleopIndexTester. */
  public teleopIndexTester(IndexSubsystem indexArgs) {

=======
>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9
public class indexRun extends CommandBase {

  private IndexSubsystem index;
  private boolean isDone = false;
  private double speed;
  
  /** Creates a new teleopIndex. */
  public indexRun(IndexSubsystem indexArgs, double speedArgs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speedArgs;
<<<<<<< HEAD
    this.index = indexArgs;
=======
>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9:src/main/java/frc/robot/commands/indexRun.java
    this.index = indexArgs;
    // Use addRequirements() here to declare subsystem dependencies.
>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
<<<<<<< HEAD
  public void initialize() {
  }
=======
  public void initialize() {}
>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleopIndex.java
    if (!(index.getBallPosition(1))){
      isDone = false;
      index.setSpeedTower(Constants.BELT_SPEED);
    } else {
      isDone = true;
    }
=======
    
    index.setSpeedTower(speed);
>>>>>>> d9ea0b1 (Added manual index in and out. Added organization of the index):src/main/java/frc/robot/commands/indexRun.java

=======
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleopIndexTester.java
    index.setSpeedTower(0.75);
=======
    
    index.setSpeedTower(speed);

>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9:src/main/java/frc/robot/commands/indexRun.java
>>>>>>> d9ea0b1a3d69e88065d9997f38966820adeae2d9
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setSpeedTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
