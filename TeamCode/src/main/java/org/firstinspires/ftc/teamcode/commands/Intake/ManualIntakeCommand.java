package org.firstinspires.ftc.teamcode.commands.Intake;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class ManualIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier supplier;
    private boolean constIntake = false;
    public double constIntakePower = 0;

    public ManualIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        this.supplier = () -> 1;

        addRequirements(intake);
    }

    public ManualIntakeCommand(IntakeSubsystem intake, DoubleSupplier supplier) {
        this.intake = intake;
        this.supplier = supplier;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (constIntake) {
            intake.intake(constIntakePower);
        } else {
            intake.intake(supplier.getAsDouble());
        }
    }

    @Override
    public void end() {
        intake.intake(0);
    }

    public void toggleConstIntake() {
        constIntake = !constIntake;
    }

    public void setConstIntake(boolean constIntake) {
        this.constIntake = constIntake;
    }

    public void setConstIntakePower(double power) {
        constIntakePower = power;
    }
}
