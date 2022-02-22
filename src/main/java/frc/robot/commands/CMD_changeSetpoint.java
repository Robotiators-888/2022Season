package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with
 * {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class CMD_changeSetpoint extends CommandBase {
    Shooter shoot;
    int change;

    public CMD_changeSetpoint(Shooter subsystem, int change) {
        addRequirements(subsystem);
        this.shoot = subsystem;
        this.change = change;
    }

    @Override
    public void initialize() {
        shoot.changeManualRPM(change);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}