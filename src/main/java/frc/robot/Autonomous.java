package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.*;

import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

//this is a helper class which provides useful methods for running autonomus

public class Autonomous {
    private Drivetrain drivetrain;

    /**
     * this is a helper class which provides useful methods for running autonomus
     * 
     * @param input drivetrain instance from robot container
     */
    public Autonomous(Drivetrain input) {
        this.drivetrain = input;

    }

    /**
     * opens and converts path wever file to a trajectory
     * 
     * @param weaverFile sting path to path weaver *.wpilib.json file
     *                   ex.(paths/YourPath.wpilib.json)
     * @return Trajectory onject from path weaver file
     */
    public Trajectory getTrajectory(String weaverFile) {
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(weaverFile);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + weaverFile, ex.getStackTrace());
        }

        return trajectory;
    }

    /**
     * returns ramsete command to drive provided trajectory
     * 
     * @param traj trajectory to follow
     * @return ramsete controller to follow trajectory
     */
    public RamseteCommand getRamset(Trajectory traj) {
        return new RamseteCommand(traj, drivetrain::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics, drivetrain::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                drivetrain::tankDriveVolts, drivetrain);

    }

}
