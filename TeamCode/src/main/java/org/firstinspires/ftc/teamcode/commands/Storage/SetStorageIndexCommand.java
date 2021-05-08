package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetStorageIndexCommand extends CommandBase {
    private final StorageSubSystem storage;
    private final double speed;

    public SetStorageIndexCommand(StorageSubSystem storage, double speed) {
        this.storage = storage;
        this.speed = speed;

        addRequirements(storage);
    }

    @Override
    public void initialize() {
        storage.index(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
