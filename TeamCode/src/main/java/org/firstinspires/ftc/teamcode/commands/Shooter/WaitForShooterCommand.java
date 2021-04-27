package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class WaitForShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double targetVelocity;

    public WaitForShooterCommand(ShooterSubsystem shooter, double targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;

        addRequirements(shooter);
    }

    @Override
    public boolean isFinished() {
        return shooter.getLeftVelocity() >= targetVelocity;
    }
}
