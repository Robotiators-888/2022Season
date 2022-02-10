package frc.robot.commands.Autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class straightLineAuto extends SequentialCommandGroup {
    public straightLineAuto(Autonomous autoHelper, Drivetrain drive) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);
        Trajectory StraightLineTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                List.of(), new Pose2d(3, 0, new Rotation2d(0)), config);

        addCommands(autoHelper.getRamset(StraightLineTrajectory).andThen(() -> drive.tankDriveVolts(0, 0)));
            
        

    }
}