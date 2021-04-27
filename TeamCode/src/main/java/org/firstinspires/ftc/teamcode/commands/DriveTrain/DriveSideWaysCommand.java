package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

public class DriveSideWaysCommand extends Command {
    DriveTrainSubsystem driveTrain;
    DoubleSupplier speedSupplier;

    public DriveSideWaysCommand(DriveTrainSubsystem driveTrain, DoubleSupplier speedSupplier) {
        this.driveTrain = driveTrain;
        this.speedSupplier = speedSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.driveLeft(speedSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
