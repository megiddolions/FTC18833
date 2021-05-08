package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterVelocityCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double velocity;

    public SetShooterVelocityCommand(ShooterSubsystem shooterSubsystem, double RPM) {
        this.shooterSubsystem = shooterSubsystem;
        velocity = RPM;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
