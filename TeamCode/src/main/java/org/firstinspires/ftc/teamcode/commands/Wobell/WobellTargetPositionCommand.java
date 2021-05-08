package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WobellTargetPositionCommand extends CommandBase {
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
