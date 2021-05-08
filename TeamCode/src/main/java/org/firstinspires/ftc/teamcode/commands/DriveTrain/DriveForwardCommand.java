package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCommand extends CommandBase {
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
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
