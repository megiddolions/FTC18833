package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveLeftDistanceCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final double distance;

    public DriveLeftDistanceCommand(DriveTrainSubsystem driveTrain, double mm) {
        this.driveTrain = driveTrain;
        this.distance = mm;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.set_for_autonomous();
        driveTrain.driveLeftDistance(distance);
    }

    @Override
    public boolean isFinished() {
        return driveTrain.isBusy();
    }
}
