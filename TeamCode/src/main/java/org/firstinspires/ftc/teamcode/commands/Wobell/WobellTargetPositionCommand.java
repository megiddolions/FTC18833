package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

public class WobellTargetPositionCommand extends Command {
    private final WobellSubsystem wobellSubsystem;

    public WobellTargetPositionCommand(WobellSubsystem wobellSubsystem) {
        this.wobellSubsystem = wobellSubsystem;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void execute() {
        wobellSubsystem.setLift(
                (wobellSubsystem.getTargetPosition() - wobellSubsystem.getCurrentPosition()) / -2000.0);
    }
}
