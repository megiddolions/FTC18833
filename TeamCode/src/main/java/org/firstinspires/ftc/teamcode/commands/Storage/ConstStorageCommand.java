package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConstStorageCommand extends CommandBase {
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
    public void end(boolean interrupted) {
        storage.index(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(power) == 0;
    }
}
