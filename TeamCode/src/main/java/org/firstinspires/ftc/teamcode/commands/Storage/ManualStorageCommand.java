package org.firstinspires.ftc.teamcode.commands.Storage;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import java.util.function.DoubleSupplier;

public class ManualStorageCommand extends Command {
    private final StorageSubSystem storage;
    private final DoubleSupplier supplier;

    public ManualStorageCommand(StorageSubSystem storage, DoubleSupplier supplier) {
        this.storage = storage;
        this.supplier = supplier;

        addRequirements(storage);
    }

    @Override
    public void execute() {
        storage.index(supplier.getAsDouble());
    }

    @Override
    public void end() {
        storage.index(0);
    }
}
