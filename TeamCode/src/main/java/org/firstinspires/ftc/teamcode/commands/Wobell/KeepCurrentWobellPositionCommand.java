package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class KeepCurrentWobellPositionCommand extends CommandBase {
    private final WobbleSubsystem wobbleSubsystem;

    public KeepCurrentWobellPositionCommand(WobbleSubsystem wobbleSubsystem) {
        this.wobbleSubsystem = wobbleSubsystem;

        addRequirements(wobbleSubsystem);
    }

    @Override
    public void initialize() {
        wobbleSubsystem.setTargetPosition(wobbleSubsystem.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
