package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import android.os.Build;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends Command {
    DriveTrainSubsystem driveTrain;
    DoubleSupplier leftSupplier;
    DoubleSupplier rightSupplier;

    public TankDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        this.driveTrain = driveTrain;
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setPower(leftSupplier.getAsDouble(), rightSupplier.getAsDouble());
    }

    @Override
    public void end() {
        driveTrain.stop();
    }
}
