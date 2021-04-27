package org.firstinspires.ftc.teamcode.commands.Shooter;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import static org.commandftc.RobotUniversal.*;

public class SetShooterSpeedCommand extends Command {
    private final ShooterSubsystem shooter;
    private final StorageSubSystem storage;
    private final double power;
    private double time;

    private enum State {
        Starting,
        Shooting,
        End
    }
    private State state;

    public SetShooterSpeedCommand(ShooterSubsystem shooter, StorageSubSystem storage, double power) {
        this.shooter = shooter;
        this.storage = storage;
        this.power = power;
        state = State.Starting;
        addRequirements(shooter, storage);
        telemetry.addData("State", () -> state);
    }

    @Override
    public void init() {
        shooter.setPower(power);
    }

    @Override
    public void execute() {
        switch (state) {
            case Starting:
                if (shooter.getLeftVelocity() <= 2500) {
                    state = State.Shooting;
                    time = opMode.getRuntime();
                    storage.index(1);
                }
                break;
            case Shooting:
                if (time <= opMode.getRuntime() - 5) {
                    storage.index(0);
                    state = State.End;
                }
                break;
        }
    }

    @Override
    protected void end() {
        shooter.setPower(0);
    }

    @Override
    public boolean isFinished() {
        if (state == State.End) {
            state = State.Starting;
            return true;
        } else {
            return false;
        }
    }
}
