package org.firstinspires.ftc.teamcode.commands.Storage;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

public class ConstStorageCommand extends Command {
    private final StorageSubSystem storage;
    public double power = 0;

    public ConstStorageCommand(StorageSubSystem storage) {
        this.storage = storage;

        addRequirements(storage);
    }

    @Override
    public void execute() {
        storage.index(power);
    }

    @Override
    protected void end() {
        storage.index(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(power) == 0;
    }
}
