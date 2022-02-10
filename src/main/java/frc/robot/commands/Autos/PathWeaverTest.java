package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous;

import frc.robot.subsystems.Drivetrain;


public class PathWeaverTest extends SequentialCommandGroup {
    public PathWeaverTest(Autonomous autoHelper, Drivetrain drive) {
        addCommands(autoHelper.getRamset(autoHelper.getTrajectory("paths/test2balll.wpilib.json")),
                    autoHelper.getRamset(autoHelper.getTrajectory("paths/test2balll_0.wpilib.json")).andThen(() -> drive.tankDriveVolts(0, 0)));
        
    }
}