package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import java.util.function.DoubleSupplier;

public class WobellLiftCommand extends Command {
    private final WobellSubsystem wobellSubsystem;
    private final DoubleSupplier supplier;


    public WobellLiftCommand(WobellSubsystem wobellSubsystem, DoubleSupplier supplier) {
        this.wobellSubsystem = wobellSubsystem;
        this.supplier = supplier;

        addRequirements(wobellSubsystem);
    }

    @Override
    public void execute() {
        wobellSubsystem.setLift(supplier.getAsDouble());
    }

    @Override
    protected void end() {
        wobellSubsystem.setLift(0);
    }
}
