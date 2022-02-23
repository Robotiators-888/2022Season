// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CMD_TeleopDrive;
import frc.robot.commands.CMD_IndexRun;
import frc.robot.subsystems.SUB_Canal;
import frc.robot.subsystems.SUB_Drivetrain;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.UDP.BallDataPacket;
import frc.robot.UDP.GenericBuffer;
import frc.robot.UDP.LimelightDataPacket;
import frc.robot.UDP.UDPReciever;
import frc.robot.commands.CMD_Aim;
import frc.robot.commands.CMD_ShooterManualRPM;
import frc.robot.commands.CMD_canalThrough;
import frc.robot.commands.CMD_changeSetpoint;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Index.States;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.commands.CMD_IntakeSpin;
import frc.robot.commands.CMD_ShooterSpin;
import frc.robot.commands.CMD_CanalRun;
import frc.robot.commands.CMD_TeleopClimber;


import frc.robot.commands.CMD_CanalZeroToOneBottom;
import frc.robot.commands.CMD_IndexBottomToTop;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        

        // The robot's subsystems and commands are defined here...
        // private final GenericBuffer<BallDataPacket> ballBuffer = new GenericBuffer<>();
        // private final GenericBuffer<LimelightDataPacket> limelightBuffer = new GenericBuffer<>();
        // private final UDPReciever<BallDataPacket> BallReciever = new UDPReciever<>(Constants.BALL_PORT,
        //                 () -> new BallDataPacket(), ballBuffer);
        // private final UDPReciever<LimelightDataPacket> limelightReciever = new UDPReciever<>(Constants.LIMELIGHT_PORT,
        //                 () -> new LimelightDataPacket(), limelightBuffer);

        private final Field2d field2d = new Field2d();

        // subsystems
        private SUB_Limelight limelight = new SUB_Limelight();
        private SUB_Shooter shoot = new SUB_Shooter();
        private SUB_Drivetrain drivetrain = new SUB_Drivetrain(field2d);
        private SUB_Intake intake = new SUB_Intake();
        private SUB_Index index = new SUB_Index();
        private Autonomous autoHelper = new Autonomous(drivetrain);
        private SUB_Canal canal = new SUB_Canal();
        private SUB_Climber climber = new SUB_Climber();

        // Controller
        private Joystick controller = new Joystick(Constants.JOYSTICK_PORT);

        JoystickButton C_aButton = new JoystickButton(controller, 1);
        JoystickButton C_bButton = new JoystickButton(controller, 2);
        JoystickButton C_xButton = new JoystickButton(controller, 3);
        JoystickButton C_yButton = new JoystickButton(controller, 4);
        POVButton C_dPadUp = new POVButton(controller, 0);
        POVButton C_dPadDown = new POVButton(controller, 180);
        POVButton C_dPadLeft = new POVButton(controller, 270);
        POVButton C_dPadRight= new POVButton(controller, 90);
        Trigger C_leftTrigger;
        Trigger C_rightTrigger;
        Trigger A_ToTopTrigger;

        // left Joystick
        private Joystick leftJoystick = new Joystick(Constants.LEFTJOYSTICK_PORT);

        JoystickButton L_button2 = new JoystickButton(leftJoystick, 2);
        JoystickButton L_button3 = new JoystickButton(leftJoystick, 3);
        JoystickButton L_button4 = new JoystickButton(leftJoystick, 4);
        JoystickButton L_button5 = new JoystickButton(leftJoystick, 5);
        JoystickButton L_Trigger = new JoystickButton(leftJoystick, 1);

        // right Joytick
        private Joystick rightJoystick = new Joystick(Constants.RIGHTSTICK_PORT);

        JoystickButton R_button5 = new JoystickButton(rightJoystick, 3);
        JoystickButton R_button6 = new JoystickButton(rightJoystick, 4);
        JoystickButton R_button3 = new JoystickButton(rightJoystick, 5);
        JoystickButton R_button4 = new JoystickButton(rightJoystick, 6);
        JoystickButton R_trigger = new JoystickButton(rightJoystick, 1);



        // Auto objects
        SendableChooser<Command> chooser = new SendableChooser<>();
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics);
        Trajectory ballin1 = autoHelper.getTrajectory("paths/output/test2balll.wpilib.json");
        Trajectory ballin2 = autoHelper.getTrajectory("paths/output/test2balll_0.wpilib.json");
        Trajectory onePath = autoHelper.getTrajectory("paths/output/onepathwonder.wpilib.json");
        Trajectory Str8 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(1, 0)), new Pose2d(2, 0, new Rotation2d(0)), config);

        // Auto command groups
        Command straightAuto = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(Str8.getInitialPose())),
                        autoHelper.getRamset(Str8));

        Command pwtest = new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setPosition(ballin1.getInitialPose())),
                        autoHelper.getRamset(ballin1),
                        autoHelper.getRamset(ballin2).andThen(() -> drivetrain.tankDriveVolts(0, 0)));

        Command onePathWonder = new SequentialCommandGroup(new ParallelRaceGroup(
                        new ParallelCommandGroup(new CMD_IndexRun(index, Constants.BELT_SPEED),
                                        new CMD_CanalRun(canal, -Constants.BELT_SPEED)),
                        new SequentialCommandGroup(
                                        new InstantCommand(() -> drivetrain.setPosition(onePath.getInitialPose())),
                                        autoHelper.getRamset(onePath).andThen(() -> drivetrain.tankDriveVolts(0, 0)))),
                        new ParallelCommandGroup(new CMD_IndexRun(index, Constants.BELT_SPEED),
                                        new CMD_ShooterSpin(shoot, 0.50)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                LiveWindow.disableAllTelemetry();
                configureButtonBindings();

                field2d.getObject("traj").setTrajectory(Str8);

                chooser.setDefaultOption("Simple Auto", straightAuto);
                chooser.addOption("Complex Auto", pwtest);
                chooser.addOption("one Path Wonder", onePathWonder);

                // BallReciever.start();
                // limelightReciever.start();
                SmartDashboard.putData("chooser", chooser);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                //drivetrain
                drivetrain.setDefaultCommand(new CMD_TeleopDrive(drivetrain, () -> leftJoystick.getRawAxis(1),
                                () -> rightJoystick.getRawAxis(1)));
                L_button2.whenPressed(new InstantCommand(drivetrain::toggleReverse, drivetrain));

                //climber                
                C_leftTrigger = new Trigger(() -> (controller.getRawAxis(2) > 0.5));
                C_rightTrigger = new Trigger(() -> (controller.getRawAxis(3) > 0.5));

                C_leftTrigger.whileActiveContinuous(new CMD_TeleopClimber(climber, -0.25));
                C_rightTrigger.whileActiveContinuous(new CMD_TeleopClimber(climber, 0.25));

                //Intake
                L_button4.whenPressed(new InstantCommand(intake::pistonToggle, intake));
<<<<<<< HEAD
                L_Trigger.whileHeld(new ParallelCommandGroup(new CMD_IntakeSpin(intake, 0.75), new CMD_CanalZeroToOneBottom(canal,index)));
=======
                L_Trigger.whileHeld(new ParallelCommandGroup(new CMD_IndexSpin(intake, 0.75), new CMD_CanalZeroToOneBottom(canal, index)));
>>>>>>> 879c7b6b10162e3be0de7adafebd536e8ba08a82

                //Canal
                C_dPadUp.whileHeld(new CMD_CanalRun(canal, -0.75));
                C_dPadDown.whileHeld(new CMD_CanalRun(canal, 0.75));
                C_dPadLeft.whileHeld(new CMD_canalThrough(canal, 0.75));
                C_dPadRight.whileHeld(new CMD_canalThrough(canal, -0.75));

                //Index
                index.setDefaultCommand(new CMD_IndexBottomToTop(canal, index));
                C_aButton.whileHeld(new ParallelCommandGroup(new CMD_IndexRun(index, -0.75), new CMD_ShooterSpin(shoot, 0.25)));
                C_bButton.whileHeld(new CMD_IndexRun(index, 0.75));


                //shooter
                R_button3.whileHeld(new CMD_changeSetpoint(shoot, -500));
                R_button4.whileHeld(new CMD_changeSetpoint(shoot, -100));
                R_button5.whileHeld(new CMD_changeSetpoint(shoot, 500));
                R_button6.whileHeld(new CMD_changeSetpoint(shoot, 100));
                R_trigger.whileHeld(new CMD_ShooterManualRPM(shoot));
                //L_button2 auto aim and shoot
                //C_yButton change auto target (high or low goal)


                // Index Trigger
                A_ToTopTrigger = new Trigger(() -> (index.currentState==States.ONE_BALL_BOTTOM));

    // Auto objects
   
        }
                                


    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}