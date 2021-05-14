package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetDriveSpeedModifierCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final double modifier;

    public SetDriveSpeedModifierCommand(DriveTrainSubsystem driveTrain, double modifier) {
        this.driveTrain = driveTrain;
        this.modifier = modifier;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.setPower(modifier);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
