package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualStorageCommand extends CommandBase {
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
    public void end(boolean interrupted) {
        storage.index(0);
    }
}
