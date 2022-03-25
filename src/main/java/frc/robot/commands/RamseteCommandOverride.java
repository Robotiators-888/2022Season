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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.RamseteCommandOverride;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class RamseteCommandOverride extends RamseteCommand {

  // super(t, p, r, s,k, ds, p1, p2,drivetrain::tankDriveVolts,drivetrain);
  // //RamseteCommand(r,s,k,p1,p2,d);
  // //TODO Auto-generated constructor stub
  // }

  public RamseteCommandOverride(Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController, BiConsumer<Double, Double> outputVolts,
      Drivetrain drivetrain) {
    super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController,
        outputVolts, drivetrain);
    // TODO Auto-generated constructor stub
  }

  @Override
  public void end(boolean interrupted) {

  }
}
