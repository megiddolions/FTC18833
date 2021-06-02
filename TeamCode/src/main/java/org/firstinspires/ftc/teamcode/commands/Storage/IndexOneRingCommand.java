package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexOneRingCommand extends CommandBase {
    private final StorageSubSystem storage;

    public IndexOneRingCommand(StorageSubSystem storage) {
        this.storage = storage;

        addRequirements(storage);
    }

    @Override
    public void initialize() {
        storage.set_for_autonomous();
        storage.index_distance(110);
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
