package org.firstinspires.ftc.teamcode.commands.Storage;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;

public class AutomaticStorageCommand extends Command {
    private enum StorageState {
        Waiting(false, true),
        Start(true, false),
        Middle(true, true),
        End(true, false);

        final boolean active;
        final boolean seen_ring;

        StorageState(boolean active, boolean seen_ring) {
            this.active = active;
            this.seen_ring = seen_ring;
        }

        StorageState next(boolean seeing_ring) {
            if (this.seen_ring == seeing_ring) {
                switch (this) {
                    case Waiting:
                        return Start;
                    case Start:
                        return Middle;
                    case Middle:
                        return End;
                    case End:
                        return Waiting;
                }
            }
            return this;
        }
    }

    StorageSubSystem storage;
    private StorageState storageState;

    public AutomaticStorageCommand(StorageSubSystem storage) {
        this.storage = storage;

        addRequirements(storage);

        storageState = StorageState.Waiting;
    }

    @Override
    public void execute() {
        storageState = storageState.next(storage.seeing_ring());

        if (storageState.active) {
            storage.index(1);
        } else {
            storage.index(0);
        }
    }

    protected void end() {
        storage.index(0);
        // reset storage state
        storageState = StorageState.Waiting;
    }
}
