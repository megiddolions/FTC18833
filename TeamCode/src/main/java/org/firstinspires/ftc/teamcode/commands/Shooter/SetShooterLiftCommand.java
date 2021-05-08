package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterLiftCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double change;

    public SetShooterLiftCommand(ShooterSubsystem shooter, double change) {
        this.shooter = shooter;
        this.change = change;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLift(shooter.getLift() + change);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
