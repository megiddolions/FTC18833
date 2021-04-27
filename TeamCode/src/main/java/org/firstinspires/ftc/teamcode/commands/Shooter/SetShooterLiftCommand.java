package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class SetShooterLiftCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double change;

    public SetShooterLiftCommand(ShooterSubsystem shooter, double change) {
        this.shooter = shooter;
        this.change = change;

        addRequirements(shooter);
    }

    @Override
    public void init() {
        shooter.setLift(shooter.getLift() + change);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
