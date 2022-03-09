// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_limeCheck extends CommandBase {
    Limelight limelight;

    public CMD_limeCheck(Limelight limein) {
        this.limelight = limein;
        addRequirements(limein);
    }

    @Override
    public void initialize() {
        limelight.setLed(3);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("WithinRange?", !(limelight.getDistance() > Constants.MAX_RANGE));
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
