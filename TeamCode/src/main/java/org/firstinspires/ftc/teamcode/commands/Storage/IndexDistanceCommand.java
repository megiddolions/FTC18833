package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexDistanceCommand extends CommandBase {
    private final StorageSubSystem storage;
    private final double distance;

    public IndexDistanceCommand(StorageSubSystem storage, double distance) {
        this.storage = storage;
        this.distance = distance;

        addRequirements(storage);
    }

    @Override
    public void initialize() {
        storage.set_for_autonomous();
        storage.index_distance(distance);
    }

    @Override
    public boolean isFinished() {
        return !storage.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        storage.set_for_commands();
    }
}
