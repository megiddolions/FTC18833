package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLiftPositionCommand extends CommandBase {
    private final WobellSubsystem wobellSubsystem;
    private final int target_position;

    public SetLiftPositionCommand(WobellSubsystem wobellSubsystem, int target_position) {
        this.wobellSubsystem = wobellSubsystem;
        this.target_position = target_position;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void initialize() {
        wobellSubsystem.setTargetPosition(target_position);
    }

    @Override
    public void execute() {
        wobellSubsystem.setLift((wobellSubsystem.getTargetPosition() - wobellSubsystem.getCurrentPosition()) / 2000.0);
    }

    @Override
    public void end(boolean interrupted) {
        wobellSubsystem.setLift(0);
    }
}
