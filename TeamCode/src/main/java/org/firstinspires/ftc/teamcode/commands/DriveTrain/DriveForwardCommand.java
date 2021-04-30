package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

public class DriveForwardCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final DoubleSupplier supplier;

    public DriveForwardCommand(DriveTrainSubsystem driveTrain, DoubleSupplier supplier) {
        this.driveTrain = driveTrain;
        this.supplier = supplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setPower(supplier.getAsDouble(), supplier.getAsDouble());
    }

    @Override
    protected void end() {
        driveTrain.stop();
    }
}
