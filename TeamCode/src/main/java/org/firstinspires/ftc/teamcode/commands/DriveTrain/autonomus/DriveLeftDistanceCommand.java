package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

public class DriveLeftDistanceCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final double distance;

    public DriveLeftDistanceCommand(DriveTrainSubsystem driveTrain, double mm) {
        this.driveTrain = driveTrain;
        this.distance = mm;

        addRequirements(driveTrain);
    }

    @Override
    public void init() {
        driveTrain.set_for_autonomous();
        driveTrain.driveLeftDistance(distance);
    }

    @Override
    protected void end() {
        driveTrain.set_for_commands();
    }

    @Override
    public boolean isFinished() {
        return driveTrain.isBusy();
    }
}
