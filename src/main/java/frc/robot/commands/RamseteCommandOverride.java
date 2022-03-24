package frc.robot.commands;


import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.RamseteCommandOverride;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import java.util.function.Supplier;

    public class RamseteCommandOverride extends RamseteCommand{
        private Drivetrain drivetrain;
        public RamseteCommandOverride(
            Trajectory t,
            Supplier<Pose2d> p,
            RamseteController r,
            SimpleMotorFeedforward s,
            Supplier<DifferentialDriveWheelSpeeds>  ds,
            DifferentialDriveKinematics k,
            PIDController p1,
            PIDController p2,
            Drivetrain drivetrain
            ) {
            super(t, p, r, s,k, ds, p1, p2,drivetrain::tankDriveVolts,drivetrain);
            //RamseteCommand(r,s,k,p1,p2,d);
            //TODO Auto-generated constructor stub
        }

        @Override
        public void end(boolean interrupted) {
            
          }
    }  

