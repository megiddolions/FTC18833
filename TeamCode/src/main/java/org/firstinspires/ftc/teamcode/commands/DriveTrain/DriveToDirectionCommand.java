package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToDirectionCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final DoubleSupplier verticalSupplier;
    private final DoubleSupplier horizontalSupplier;
    private final DoubleSupplier rotationSupplier;

    public DriveToDirectionCommand(DriveTrainSubsystem driveTrain, DoubleSupplier verticalSupplier, DoubleSupplier horizontalSupplier, DoubleSupplier rotationSupplier) {
        this.driveTrain = driveTrain;
        this.verticalSupplier = verticalSupplier;
        this.horizontalSupplier = horizontalSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.holonomicDrive(horizontalSupplier.getAsDouble(), verticalSupplier.getAsDouble(), rotationSupplier.getAsDouble(), 1);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
