package org.firstinspires.ftc.teamcode.opmods.autonomous;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

public abstract class DefaultAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected StorageSubSystem storage;
    protected IntakeSubsystem intake;

    @Override
    public void plan() {

    }
}
