package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import static org.commandftc.RobotUniversal.*;

public class SetShooterSpeedCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double power;
    private double time;

    public SetShooterSpeedCommand(ShooterSubsystem shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void init() {
        shooter.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
