package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WobellLiftCommand extends CommandBase {
    private final WobbleSubsystem wobbleSubsystem;
    private final DoubleSupplier supplier;


    public WobellLiftCommand(WobbleSubsystem wobbleSubsystem, DoubleSupplier supplier) {
        this.wobbleSubsystem = wobbleSubsystem;
        this.supplier = supplier;

        addRequirements(wobbleSubsystem);
    }

    @Override
    public void execute() {
        wobbleSubsystem.setLift(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        wobbleSubsystem.setLift(0);
    }
}
