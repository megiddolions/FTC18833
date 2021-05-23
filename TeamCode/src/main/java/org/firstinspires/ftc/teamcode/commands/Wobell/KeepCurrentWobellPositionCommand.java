package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class KeepCurrentWobellPositionCommand extends CommandBase {
    private final WobellSubsystem wobellSubsystem;

    public KeepCurrentWobellPositionCommand(WobellSubsystem wobellSubsystem) {
        this.wobellSubsystem = wobellSubsystem;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void initialize() {
        wobellSubsystem.setTargetPosition(wobellSubsystem.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
