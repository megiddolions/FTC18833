package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

public class SetLiftPositionCommand extends Command {
    private final WobellSubsystem wobellSubsystem;
    private final int target_position;

    public SetLiftPositionCommand(WobellSubsystem wobellSubsystem, int target_position) {
        this.wobellSubsystem = wobellSubsystem;
        this.target_position = target_position;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void init() {
        wobellSubsystem.setTargetPosition(target_position);
    }

    @Override
    public void execute() {
        wobellSubsystem.setLift((wobellSubsystem.getTargetPosition() - wobellSubsystem.getCurrentPosition()) / 2000.0);
    }

    @Override
    protected void end() {
        wobellSubsystem.setLift(0);
    }
}
