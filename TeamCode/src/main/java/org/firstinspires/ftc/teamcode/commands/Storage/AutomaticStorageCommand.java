package org.firstinspires.ftc.teamcode.commands.Storage;

import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutomaticStorageCommand extends CommandBase {
    private final StorageSubSystem storage;
    private final Command index_command;

    public AutomaticStorageCommand(StorageSubSystem storage) {
        this.storage = storage;
        index_command = new WaitCommand(0.2).andThen(new IndexDistanceCommand(storage, 115));

        addRequirements(storage);
    }

    @Override
    public void execute() {
        if (!index_command.isScheduled() && storage.seeing_ring()) {
            index_command.schedule();
        }
    }
}
