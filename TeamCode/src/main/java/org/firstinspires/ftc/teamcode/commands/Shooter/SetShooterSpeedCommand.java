package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterSpeedCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double power;
    private double time;

    public SetShooterSpeedCommand(ShooterSubsystem shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
