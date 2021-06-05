package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLiftPositionCommand extends CommandBase {
    private final WobbleSubsystem wobbleSubsystem;
    private final int target_position;

    public SetLiftPositionCommand(WobbleSubsystem wobbleSubsystem, int target_position) {
        this.wobbleSubsystem = wobbleSubsystem;
        this.target_position = target_position;

        addRequirements(wobbleSubsystem);
    }

    @Override
    public void initialize() {
        wobbleSubsystem.setTargetPosition(target_position);
    }

    @Override
    public void execute() {
        wobbleSubsystem.setLift((wobbleSubsystem.getTargetPosition() - wobbleSubsystem.getCurrentPosition()) / 2000.0);
    }

    @Override
    public void end(boolean interrupted) {
        wobbleSubsystem.setLift(0);
    }
}
