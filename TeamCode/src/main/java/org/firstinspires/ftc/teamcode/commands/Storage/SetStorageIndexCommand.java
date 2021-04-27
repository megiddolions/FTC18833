package org.firstinspires.ftc.teamcode.commands.Storage;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

public class SetStorageIndexCommand extends Command {
    private final StorageSubSystem storage;
    private final double speed;

    public SetStorageIndexCommand(StorageSubSystem storage, double speed) {
        this.storage = storage;
        this.speed = speed;

        addRequirements(storage);
    }

    @Override
    public void init() {
        storage.index(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
