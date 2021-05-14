package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    TankDrive driveTrain;
    DoubleSupplier leftSupplier;
    DoubleSupplier rightSupplier;

    public TankDriveCommand(TankDrive driveTrain, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        this.driveTrain = driveTrain;
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.tankDrive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
