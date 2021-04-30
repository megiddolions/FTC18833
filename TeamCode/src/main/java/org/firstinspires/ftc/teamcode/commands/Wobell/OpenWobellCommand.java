package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

public class OpenWobellCommand extends Command {
    private final WobellSubsystem wobellSubsystem;

    public OpenWobellCommand(WobellSubsystem wobellSubsystem) {
        this.wobellSubsystem = wobellSubsystem;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void init() {
        wobellSubsystem.open();
    }

    @Override
    protected void end() {
        wobellSubsystem.close();
    }
}
