package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WaitForShooterCommand extends WaitUntilCommand {
    private final ShooterSubsystem shooter;
    private final double targetVelocity;

    public WaitForShooterCommand(ShooterSubsystem shooter, double targetVelocity) {
        super(() -> shooter.getLeftVelocity() >= targetVelocity);
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;

        addRequirements(shooter);
    }
}
