// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_setDrive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private Drivetrain drive;
    private Double Left, Right;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CMD_setDrive(Drivetrain drivetrain, Double L, Double R) {
        this.drive = drivetrain;
        this.Left = L;
        this.Right = R;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.setIdleMode(IdleMode.kBrake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (drive.getReverse()) {
            drive.setMotors(Right, Left, 0.75);
        } else {
            drive.setMotors(-1 * Left, -1 * Right, 0.75);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.setMotors(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
