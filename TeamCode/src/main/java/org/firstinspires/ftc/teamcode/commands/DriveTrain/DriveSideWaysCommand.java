package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSideWaysCommand extends CommandBase {
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
