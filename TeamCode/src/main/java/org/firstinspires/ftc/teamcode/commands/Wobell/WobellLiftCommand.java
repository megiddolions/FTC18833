package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WobellLiftCommand extends CommandBase {
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
    public void end(boolean interrupted) {
        wobellSubsystem.setLift(0);
    }
}
