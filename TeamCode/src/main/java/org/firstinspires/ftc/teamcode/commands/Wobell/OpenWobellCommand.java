package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenWobellCommand extends CommandBase {
    private final WobellSubsystem wobellSubsystem;

    public OpenWobellCommand(WobellSubsystem wobellSubsystem) {
        this.wobellSubsystem = wobellSubsystem;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void initialize() {
        wobellSubsystem.open();
    }

    @Override
    public void end(boolean interrupted) {
        wobellSubsystem.close();
    }
}
