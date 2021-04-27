package org.firstinspires.ftc.teamcode.commands.Intake;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class StartIntakeCommand extends Command {
    IntakeSubsystem intake;
    DoubleSupplier supplier;

    public StartIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        this.supplier = () -> 1;

        addRequirements(intake);
    }

    public StartIntakeCommand(IntakeSubsystem intake, DoubleSupplier supplier) {
        this.intake = intake;
        this.supplier = supplier;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake(supplier.getAsDouble());
    }

    @Override
    public void end() {
        intake.intake(0);
    }
}
