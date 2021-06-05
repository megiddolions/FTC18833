package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenWobellCommand extends CommandBase {
    private final WobbleSubsystem wobbleSubsystem;

    public OpenWobellCommand(WobbleSubsystem wobbleSubsystem) {
        this.wobbleSubsystem = wobbleSubsystem;

//        addRequirements(wobellSubsystem);
    }

    @Override
    public void initialize() {
        wobbleSubsystem.open();
    }

    @Override
    public void end(boolean interrupted) {
        wobbleSubsystem.close();
    }
}
