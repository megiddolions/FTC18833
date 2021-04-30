package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.commandftc.Command;
import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import static org.commandftc.RobotUniversal.telemetry;

public class DriveForwardAutoCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final double distance;
    private boolean has_started = false;

    public DriveForwardAutoCommand(DriveTrainSubsystem driveTrain, double mm) {
        this.driveTrain = driveTrain;
        this.distance = mm;

        addRequirements(driveTrain);
    }

    @Override
    public void init() {
        driveTrain.driveForwardDistance(distance);
        has_started = true;
    }

    @Override
    public void execute() {
        telemetry.addData("Command", this.getClass().getSimpleName());
    }

    @Override
    protected void end() {
        driveTrain.stop();
        has_started = false;
    }

    @Override
    public boolean isFinished() {
        return has_started && !driveTrain.isBusy();
    }
}
